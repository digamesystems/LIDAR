/*
    Shuttle_Basestation
    
    An application to log data from directional people-counters on shuttle 
    buses. Reports rider In/Out behavior for the entire shuttle route to the
    Parkdata System when a shuttle returns within range of a specific WiFi SSID
    desigated as the "reportLocation".

    As the bus travels its route, WiFi SSIDs are used to track location. If 
    events are logged at an unknown location a pseudo location starting with 'UN' 
    is created.
    
    Written for the ESP32 WROOM Dev board V4 
    (Incl. WiFi, Bluetooth, and stacks of I/O.)
    
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/hw-reference/esp32c3/user-guide-devkitc-02.html

    Copyright 2021, Digame Systems. All rights reserved.
*/
#define SW_VERSION "1.0.0"
#define INBOUND  0  // The direction of counter events
#define OUTBOUND 1
#define NUM_COUNTERS 2
#define SHUTTLE 0 // Friendlier names for indicies
#define TRAILER 1

#include <digameDebug.h>      // Serial debugging defines. 
#include <digameFile.h>       // SPIFFS file Handling
#include <digameTime.h>       // Time functions for RTC, NTP, etc. 
#include <digameNetwork_v2.h> // For connections, MAC Address and reporting.
#include <digameDisplay.h>    // eInk Display support.

#include "BluetoothSerial.h"  // Virtual UART support for Bluetooth Classic 
#include <WiFi.h>           
#include <ArduinoJson.h>     
#include <CircularBuffer.h>   // Adafruit library for handling circular buffers of data. 

const int samples = 100;

CircularBuffer<String *, samples> shuttleStops; // A buffer containing pointers to JSON messages to be 
                                                // sent to the LoRa basestation or server.

NetworkConfig networkConfig; // See digameNetwork_v2.h for structure definition.

BluetoothSerial btUART1; // Create a BlueTooth Serial port to talk to the counter
BluetoothSerial btUART2; // Create a BlueTooth Serial port to talk to the counter

struct ShuttleStop { 
  String location       = "UNK";  
  String startTime      = "00:00:00"; // When we got to this location
  String endTime        = "00:00:00"; // When we last saw an event while here
  String counterMACAddresses[NUM_COUNTERS]    = {"",""};  // TODO: Consider moving into the "BaseStation"
  unsigned int counterEvents[NUM_COUNTERS][2] = {{0, 0},  // Counter 0, in/out count
                                                 {0, 0}}; // Counter 1, in/out count                              
};

ShuttleStop currentShuttleStop;

String routeName   = "Route 1";
String shuttleName = "Shuttle 6";

String reportingLocation         = "AndroidAP3AE2";
String reportingLocationPassword = "ohpp8971";

String knownLocations[] = {"_Bighead", 
                          "Tower88", 
                          "William Shatner's Toupee", 
                          "Pretty Fly For A Wi-Fi V4",
                          "_4th_StreetPizzaCo", 
                          "SanPedroSquareMarket",
                          "AndroidAP3AE2"};

String counterNames[]     = {"ShuttleCounter_c610", "ShuttleCounter_5ccc"}; 

uint8_t counter1Address[] = {0xAC,0x0B,0xFB,0x25,0xC6,0x12};
uint8_t counter2Address[] = {0xE8,0x68,0xE7,0x30,0xAA,0x0E};

String currentLocation  = "UNK"; 
String previousLocation = "UNK";

String titleToDisplay = "";
String textToDisplay = "";


// Multi-Tasking
SemaphoreHandle_t mutex_v;     // Mutex used to protect variables across RTOS tasks. 
TaskHandle_t eInkManagerTask;  // A task to update the eInk display.

// Utility Function: Number of items in an array
#define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

// WiFi
String scanForKnownLocations(String knownLocations[], int arraySize);

// UI
void showSplashScreen();
void showCountDisplay(ShuttleStop &shuttleStop);
void eInkManager(void *parameter); // TASK that runs on Core 0

// Initialization
void configureIO();
void configureDisplay();
void configureRTC();
void configureWiFi();
void configureBluetooth(); 
void configureEinkManagerTask();
void loadParameters();

// Bluetooth Counters
bool connectToCounter(BluetoothSerial &btUART, String counter);
bool connectToCounterAddr(BluetoothSerial &btUART, uint8_t counterAddr[]);

String sendReceive(BluetoothSerial &btUART, String stringToSend, int counterID);

// ShuttleStop wants to be a class...
void   pushShuttleStop(ShuttleStop &shuttleStop);
void   resetShuttleStop(ShuttleStop &shuttleStop);
void   updateShuttleStop(ShuttleStop &shuttleStop, String jsonMessage, int counterNumber);
String getShuttleStopJSON(ShuttleStop &shuttleStop);


void processLocationChange(ShuttleStop &currentShuttleStop);


//****************************************************************************************
// SETUP - Device initialization                                   
//****************************************************************************************
void setup(){
  
  Serial.begin(115200);   // Intialize terminal serial port
  delay(1000);            // Give port time to initalize

  mutex_v = xSemaphoreCreateMutex(); // The mutex we will use to protect variables 
                                     // across tasks

  loadParameters();
  showSplashScreen();
  
  configureIO();
  configureDisplay();
  configureRTC();
  configureWiFi();
  configureBluetooth();

  // Setup our counter. 
  sendReceive(btUART1, "-",0); // Turn off the menu system
  sendReceive(btUART1, "g",0); // Get a value
  sendReceive(btUART1, "c",0); // Clear the counter
  
  resetShuttleStop(currentShuttleStop);
  
  configureEinkManagerTask();
  
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("RUNNING!");
  DEBUG_PRINT("mainLoop Running on Core #: ");
  DEBUG_PRINTLN(xPortGetCoreID());
  DEBUG_PRINTLN();
}


//****************************************************************************************
// MAIN LOOP                                   
//****************************************************************************************
unsigned int t1=-600000,t2=t1;
unsigned int t3=0,t4=t3;
unsigned int networkScanInterval = 15000;
unsigned int counterPollingInterval = 1000;

void loop(){

  t2=millis();
  
  // Scan for known SSIDs.
  if ((t2-t1) > networkScanInterval){
    currentLocation  = scanForKnownLocations(knownLocations, NUMITEMS(knownLocations));
    DEBUG_PRINTLN("Previous Location: " + previousLocation + " Current Location: " + currentLocation);
    t1=millis(); 
  }

  if (currentLocation != previousLocation){ // We've moved
    DEBUG_PRINTLN("New Location!");
    
    String counterStats = sendReceive(btUART1, "g",0); // Get the current count
    if (counterStats !="") updateShuttleStop(currentShuttleStop, counterStats, 0);
    
    processLocationChange(currentShuttleStop);
    
    // Get ready to take data here, at the new location
    counterStats = sendReceive(btUART1, "c",0); // Clear the counter
    resetShuttleStop(currentShuttleStop);

    //titleToDisplay = "LOCATION";
    //textToDisplay = "\n " + currentLocation + "\n\n Awaiting Counts...";
  }
 
  // Poll the counters.
  t4 = millis();
  if ((t4-t3) > counterPollingInterval){
    String counterStats = sendReceive(btUART1, "g", 0);
    if (counterStats !="") updateShuttleStop(currentShuttleStop, counterStats, 0);
    t3 = millis();   
  }    
  
  delay(20);
}


//****************************************************************************************
bool countsChanged (ShuttleStop shuttleStop) // Have the shuttleStop's counters changed
                                             // since we last asked?
//****************************************************************************************
{
  static int lastInCount = 0;
  static int lastOutCount = 0;
  bool retValue = false;
  
  if ( (lastInCount  != shuttleStop.counterEvents[SHUTTLE][INBOUND]) ||
       (lastOutCount != shuttleStop.counterEvents[SHUTTLE][OUTBOUND]) ) 
  {   
    retValue = true;
    lastInCount  = shuttleStop.counterEvents[SHUTTLE][INBOUND];
    lastOutCount = shuttleStop.counterEvents[SHUTTLE][OUTBOUND];   
  }

  DEBUG_PRINTLN("countsChanged says: " + String(retValue));
  return retValue; 
  
}

bool displayTextChanged(String displayText){
  static String oldText = "";
  bool retVal = false;
  
  if (displayText != oldText){
    retVal = true;
    oldText = displayText;  
  }  

  return retVal;  
}

String rotateSpinner(){
  static String spinner = "|";
  
  //OLD SCHOOL! :)
  if (spinner == "|") {
    spinner = "/";
  } else if (spinner == "/") {
    spinner = "-";
  } else if (spinner == "-") {
    spinner = "\\";
  } else { 
    spinner = "|";
  }
  return spinner;
}

//****************************************************************************************
// A TASK that runs on Core0. Updates the eInk display with the currentShuttleStop data
// if it has changed.
//****************************************************************************************
void eInkManager(void *parameter){
  const unsigned int displayUpdateInterval = 500;
  
  for(;;){   
    
    vTaskDelay(displayUpdateInterval / portTICK_PERIOD_MS);
  
    // Check if we need an update to the display to show new counts... 
    if (countsChanged(currentShuttleStop)) {
        showCountDisplay(currentShuttleStop);
    } else if (displayTextChanged(textToDisplay)) {
      displayTextScreen(titleToDisplay,textToDisplay);  
    } 

    showPartialXY(rotateSpinner(),180,180);
    
  }
}
    


//****************************************************************************************
void deliverRouteReport(){
//****************************************************************************************
  String reportToIssue; 
  bool postSuccessful = false;

  while (shuttleStops.size() > 0){ 
    DEBUG_PRINT("Buffer Size: ");
    DEBUG_PRINTLN(shuttleStops.size());

    reportToIssue = routeReportPrefix(); 

    // FAILURE MODE: 
    // This is a nifty opportunity to get hung up. If we pull into the reporting 
    // location and someone pulls the plug on the router, we'll hang here forever looking
    // for it...

    titleToDisplay = "V. CENTER";
    textToDisplay = "\n\n   Sending\n\n   Report...";
    
    if (WiFi.status() != WL_CONNECTED){  
      while (!(WiFi.status() == WL_CONNECTED)){enableWiFi(networkConfig);}
    }

    
    String activeShuttleStop = String(shuttleStops.first()->c_str()); // Read from the buffer without removing the data from it.
    DEBUG_PRINTLN(activeShuttleStop);
    reportToIssue += activeShuttleStop;
    //if (shuttleStops.size>1) reportToIssue += ","
    reportToIssue += "]}";

    DEBUG_PRINTLN(reportToIssue);
  
    postSuccessful = postJSON(reportToIssue, networkConfig);

    if (postSuccessful) {
      String  * entry = shuttleStops.shift();
      delete entry;
      DEBUG_PRINTLN("Success!");
      DEBUG_PRINTLN();
    }

  }

  
  titleToDisplay = "V. CENTER";
  textToDisplay = "\n\n   Sending\n\n   Complete.";
  delay(2000); // Give folks a sec to read the update.
 
   
}

//****************************************************************************************
void processLocationChange(ShuttleStop &currentShuttleStop)
//****************************************************************************************
{
  // Log what happened at the last location
  if (currentShuttleStop.endTime != "00:00:00"){ // Append report structure if events are present. 
    pushShuttleStop(currentShuttleStop);
  }

  
  if (currentLocation == reportingLocation){
    DEBUG_PRINTLN("We have arrived at the reportingLocation!"); 
    deliverRouteReport();
  } else if (previousLocation == reportingLocation){
    DEBUG_PRINTLN("We have left the reportingLocation!");
  } else {
    DEBUG_PRINTLN("We have changed shuttle stops!");
  }
  
  previousLocation = currentLocation;
}


// IO Routines

//****************************************************************************************
void loadParameters(){
//****************************************************************************************
  StaticJsonDocument<2048> doc;  
 
  if(!SPIFFS.begin()){
    DEBUG_PRINTLN("    File System Mount Failed");
  } else {
    //DEBUG_PRINTLN("    SPIFFS up!");
    String temp;
    
    temp = readFile(SPIFFS, "/config.txt");
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, temp);
    if (error)
    {
      Serial.println(F("    Failed to parse file, using default configuration"));
      return;
    } 
    //DEBUG_PRINTLN((const char *)doc["shuttleName"]);  
    shuttleName               = (const char *)doc["shuttleName"];
    routeName                 = (const char *)doc["routeName"];
    reportingLocation         = (const char *)doc["reportingLocation"];
    reportingLocationPassword = (const char *)doc["reportingLocationPassword"];

    
    networkConfig.ssid      = reportingLocation;
    networkConfig.password  = reportingLocationPassword;
    networkConfig.serverURL = "http://199.21.201.53/trailwaze/zion/lidar_shuttle_import.php";
    
    //DEBUG_PRINTLN(doc["knownLocations"].size());
    
    for (int i=0; i<doc["knownLocations"].size(); i++){
      knownLocations[i] = String((const char *)doc["knownLocations"][i]);
      //DEBUG_PRINTLN(knownLocations[i]);
    }
    
    for (int i=0; i<doc["counterNames"].size(); i++){
      counterNames[i] = String((const char *)doc["counterNames"][i]);
      //DEBUG_PRINTLN(counterNames[i]);
    }
    
  }    
}


// UI Routines

//****************************************************************************************
// Program Splash / copyright.
//****************************************************************************************
void showSplashScreen(){
  DEBUG_PRINTLN("*****************************************************");
  DEBUG_PRINTLN("ParkData Shuttle Counter Base Station");
  DEBUG_PRINTLN("Version 1.0");
  DEBUG_PRINT("Device Name: ");
  DEBUG_PRINTLN(shuttleName);
  DEBUG_PRINTLN("Copyright 2021, Digame Systems. All rights reserved.");
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("               Hit <ENTER> for Menu"); 
  DEBUG_PRINTLN("*****************************************************");  
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("HARDWARE INITIALIZATION");
  DEBUG_PRINTLN();  
}


//****************************************************************************************
// Update the eInk display with the events seen at the current shuttle stop.
//****************************************************************************************
void showCountDisplay(ShuttleStop &shuttleStop){
  char stopInfo [512];
  int n;
    
  n = sprintf(stopInfo, "In %d Out %d",
            shuttleStop.counterEvents[SHUTTLE][INBOUND],
            shuttleStop.counterEvents[SHUTTLE][OUTBOUND]);

  if (shuttleStop.location == reportingLocation){
    displayTextScreenLarge("V. Center", stopInfo);
  } else {
    displayTextScreenLarge(shuttleStop.location.c_str(), stopInfo);
  }
               
}

// WiFI 


//****************************************************************************************
// Scans to see if we are at a known location. This version uses WiFi networks. 
// Here, we return the strongest known SSID we can see. If we don't see any, we return
// "UNK HH:MM:SS"
//****************************************************************************************
String scanForKnownLocations(String knownLocations[], int arraySize){

  char strTime[12];
  sprintf (strTime, "%02d:%02d:%02d", getRTCHour(),getRTCMinute(),getRTCSecond());
  
  String retValue = "UNK " + String(strTime);

  for (int attempts = 0; attempts < 3; attempts++){ // scan for known networks up to n times...
    DEBUG_PRINTLN("Scanning... Attempts = " + String(attempts));
    
    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
  
    if (n == 0) {
      DEBUG_PRINTLN(" No networks found.");
    } else {
      for (int i = 0; i < n; ++i) {
        // Return the first match to our known SSID list -- Return values are ordered by RSSI
        for (int j = 0; (j<arraySize); j++){
          if (WiFi.SSID(i) == knownLocations[j].c_str()){
            retValue = knownLocations[j];
            return retValue; // If we find a known network, return it and exit.
          }
        }       
      }
    }
  }
  DEBUG_PRINTLN("*******************************************");
  DEBUG_PRINTLN("No Known Networks found!");
  return retValue; // Return the "unknown" location
}


//****************************************************************************************
// Configuration routines called during startup.
//****************************************************************************************

void configureIO(){
  pinMode(0, OUTPUT);
  pinMode(2, INPUT);
  digitalWrite(0, HIGH);   // turn the pin
}

void configureDisplay(){
  initDisplay();
  displayTitles("Parkdata", "Shuttle Bus");
  centerPrint("Passenger", 65);
  centerPrint("Counting System", 85);
  //centerPrint("Version", 105);
  centerPrint(SW_VERSION, 105);
  displayCopyright();
}


void configureRTC(){  
  DEBUG_PRINTLN(" RTC...");
  initRTC();
  currentShuttleStop.startTime = getRTCTime();
}

void configureWiFi(){
  DEBUG_PRINT(" WiFi... ");
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  DEBUG_PRINTLN(" OK.");
  delay(100);  
}

void configureBluetooth(){  
  DEBUG_PRINTLN(" Bluetooth...");
  btUART2.begin("ShuttleBasestationB", true); // Bluetooth device name 
                                            //  TODO: Provide opportunity to change names. 
  delay(500); // Give port time to initalize
  btUART1.begin("ShuttleBasestationA", true); // Bluetooth device name 
                                            //  TODO: Provide opportunity to change names. 
  delay(500); // Give port time to initalize
}


void configureEinkManagerTask(){
  // Set up a separate task to update the eInk display. 
  // Task that will be executed in the eInkManager() function with priority 0 and 
  // executed on core 0.
  
  xTaskCreatePinnedToCore(
    eInkManager,      // Task function. 
   "eInk Manager",    // name of task. 
    10000,            // Stack size of task 
    NULL,             // parameter of the task 
    0,                // priority of the task 
    &eInkManagerTask, // Task handle to keep track of created task 
    0);               // pin task to core 0   
}


bool connectToCounter(BluetoothSerial &btUART, String counter){ 
// Comment from the library author: 
// connect(address) is fast (upto 10 secs max), connect(name) is slow (upto 30 secs max) as it needs
// to resolve name to address first, but it allows to connect to different devices with the same name.
// Set CoreDebugLevel to Info to view devices bluetooth address and device names

  initDisplay();
  displayTitles("CONNECTING", "Searching...");
  centerPrint("Connecting to:", 70);
  centerPrint(counter, 90);
  displayCopyright();

  //displayTextScreen("CONNECTING","\n ...SEARCHING...");
  //DEBUG_PRINT("Connecting to " + counter + "...");
  bool connected = btUART.connect(counter);
  
  if(connected) {
    DEBUG_PRINTLN(" Success! Awaiting Counts...");
    //displayTextScreen("CONNECTED","\n    SUCCESS!!!\n\n Awaiting Counts\n\n       ...");
    initDisplay();
    displayTitles("CONNECTED", "");
    centerPrint("SUCCESS!", 70);
    centerPrint("Awaiting counts...", 90);
    displayCopyright();

  } else {
    DEBUG_PRINTLN(" Failed to connect."); 
    displayTextScreen("CONNECT",  "\n     FAILED!");
  } 

  return connected; 
}


bool connectToCounterAddr(BluetoothSerial &btUART, uint8_t counterAddr[] ){ 
// Comment from the library author: 
// connect(address) is fast (upto 10 secs max), connect(name) is slow (upto 30 secs max) as it needs
// to resolve name to address first, but it allows to connect to different devices with the same name.
// Set CoreDebugLevel to Info to view devices bluetooth address and device names
  
  DEBUG_PRINT("Connecting to address...");
  bool connected = btUART.connect(counterAddr);
  
  if(connected) {
    DEBUG_PRINTLN(" Success!");
  } else {
    DEBUG_PRINTLN(" Failed to connect."); 
  } 

  return connected; 
}

String sendReceive(BluetoothSerial &btUART, String stringToSend, int counterID){
  unsigned int timeout = 2000;
  bool timedOut = false; 
  unsigned long t1, t2;
  String inString="";
  bool btConnected = false; 
  int maxRetries = 10;
  int retries = 0; 
  static int lastID = 0; 
  
  unsigned long t3 = millis();
  
  if ((lastID!=counterID)||(btUART.connected()==false)){

    if (btUART.connected()){ btUART.disconnect();}
    
    // Give ourselves a couple attempts to make a connection. 
    while ((retries<maxRetries)&&(!(btConnected))){
      DEBUG_PRINTLN("TRY: " + String(retries));
  
      //Lookup by name
      btConnected = connectToCounter(btUART, counterNames[counterID]);
  
      //Lookup by addr (seems less reliable for some reason...)
      /*
      if (counterID == SHUTTLE){
        btConnected = connectToCounterAddr(btUART, counter1Address);
      } else if (counterID == TRAILER){
        btConnected = connectToCounterAddr(btUART, counter2Address);
      }

      */
      retries++;
    }
  }

  
  if (btUART.connected()){
    
    if (btUART.available()){
      String junkString = btUART.readString();
      DEBUG_PRINTLN("Junk: " + junkString);  
    }

    btUART.flush();
    btUART.println(stringToSend);
    
    t1 = millis();
    t2 = t1;
    
    while ( (!(btUART.available())) && (!(timedOut)) ){
      delay(10);
      t2 = millis();
      timedOut = ((t2-t1) > timeout );
    }  

    if (timedOut){ 
      return inString; 
    }
    
    if ( btUART.available() ){ 
      //while ( btUART.available() ){  // read until we are caught up.
        inString = btUART.readStringUntil('\n');
      //}

      if (btUART.available()){
        DEBUG_PRINTLN("EXTRA STUFF!!!!");
      }
       
      inString.trim();
      //DEBUG_PRINTLN("Reply to:  " + stringToSend);
      DEBUG_PRINTLN(getRTCTime() + " " + inString);
        
    }
    
  
  } else {
    DEBUG_PRINTLN("Problem connecting to counter.");  
  }

  unsigned long t4 = millis();

  //DEBUG_PRINTLN("Time for sendReceive: " + String(t4-t3));
  return inString;  
  
}

// Data Management TODO: Turn much of this into a couple of classes

//****************************************************************************************
// Initialize the shuttleStop structure. Called when we leave a stop.                                    
//****************************************************************************************
void resetShuttleStop(ShuttleStop &shuttleStop){
  shuttleStop.location = currentLocation;
  shuttleStop.startTime = getRTCTime();
  shuttleStop.endTime = "00:00:00"; //shuttleStop.startTime;
  shuttleStop.counterEvents[SHUTTLE][INBOUND] = 0;
  shuttleStop.counterEvents[SHUTTLE][OUTBOUND] = 0;
  shuttleStop.counterEvents[TRAILER][INBOUND] = 0;
  shuttleStop.counterEvents[TRAILER][OUTBOUND] = 0;
}


//****************************************************************************************
// Update the shuttleStop with data from one of the counters. 'message' is the JSON 
// string the counters send us over Bluetooth.                                   
//****************************************************************************************
void updateShuttleStop(ShuttleStop &shuttleStop, String jsonMessage, int counterNumber){
  StaticJsonDocument<2048> counterMessage;  
  
  DeserializationError err = deserializeJson(counterMessage, jsonMessage);

  if (err){
    DEBUG_PRINTLN("deserializeJson() failed.");
    return;  
  }
  
  unsigned int inCount = atoi((const char *)counterMessage["inbound"]);
  unsigned int outCount = atoi((const char *)counterMessage["outbound"]);

  if ((inCount  != shuttleStop.counterEvents[counterNumber][INBOUND]) || 
      (outCount != shuttleStop.counterEvents[counterNumber][OUTBOUND])) 
  
  {
    shuttleStop.endTime = getRTCTime();
    shuttleStop.counterMACAddresses[counterNumber]= (const char *)counterMessage["deviceMAC"];
    shuttleStop.counterEvents[counterNumber][INBOUND] = inCount;
    shuttleStop.counterEvents[counterNumber][OUTBOUND] = outCount;
  }
}


//****************************************************************************************
// Dump the base station struct in JSON                                  
//****************************************************************************************
String getShuttleStopJSON(ShuttleStop &shuttleStop){   
  StaticJsonDocument<2048> shuttleStopJSON;
  
  shuttleStopJSON["location"]                          = shuttleStop.location;
  shuttleStopJSON["startTime"]                         = shuttleStop.startTime;
  shuttleStopJSON["endTime"]                           = shuttleStop.endTime;
  shuttleStopJSON["sensors"]["shuttle"]["macAddress"]  = shuttleStop.counterMACAddresses[SHUTTLE];
  shuttleStopJSON["sensors"]["shuttle"]["inbound"]     = shuttleStop.counterEvents[SHUTTLE][INBOUND];
  shuttleStopJSON["sensors"]["shuttle"]["outbound"]    = shuttleStop.counterEvents[SHUTTLE][OUTBOUND];
  shuttleStopJSON["sensors"]["trailer"]["macAddress"]  = shuttleStop.counterMACAddresses[TRAILER];
  shuttleStopJSON["sensors"]["trailer"]["inbound"]     = shuttleStop.counterEvents[TRAILER][INBOUND];
  shuttleStopJSON["sensors"]["trailer"]["outbound"]    = shuttleStop.counterEvents[TRAILER][OUTBOUND];

  //Serial.print("Base Station State: ");
  String retValue; 
  //serializeJsonPretty(shuttleStopJSON, retValue);
  serializeJson(shuttleStopJSON, retValue);
  
  return retValue;
}


//****************************************************************************************
// Prefix for the JSON shuttle route report.                                  
//****************************************************************************************
String routeReportPrefix(){
   
    String prefix = "{\"routeName\":\"" + routeName    + "\"," +
                    "\"shuttleName\":\"" + shuttleName    + "\"," + 
                    "\"macAddress\":\"" + getMACAddress() + "\"," +
                    "\"shuttleStops\":["; 
    return prefix;
}


//****************************************************************************************
// Add the results from a shuttle stop to the circular buffer.                                 
//****************************************************************************************
void pushShuttleStop(ShuttleStop &shuttleStop){
  String shuttleStopJSON = getShuttleStopJSON(shuttleStop);  
  String * msgPtr = new String(shuttleStopJSON);
  shuttleStops.push(msgPtr);
}
