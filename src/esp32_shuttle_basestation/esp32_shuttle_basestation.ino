/*
    Shuttle_Basestation
    
    An application to log data from directional people-counters on shuttle 
    buses. Reports rider In/Out behavior for the entire shuttle route to the
    Parkdata System when a shuttle returns within range of a specific WiFi SSID
    desigated as the "reportLocation".

    As the bus travels its route, WiFi SSIDs are used to track location.
    
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
#include <digamePowerMgt.h>
#include <digameNetwork_v2.h> // For connections, MAC Address and reporting.
#include <digameDisplay.h>    // eInk Display support.
#include "BluetoothSerial.h"  // Virtual UART support for Bluetooth Classic 
#include <WiFi.h>           
#include <ArduinoJson.h>     

NetworkConfig networkConfig; // See digameNetwork_v2.h Currently using defaults. 
                             // TODO: initialize with what we want...

BluetoothSerial btUART1; // Create a BlueTooth Serial port to talk to the counter
BluetoothSerial btUART2; // Create a BlueTooth Serial port to talk to the counter

struct ShuttleStop { 
  String location       = "Unknown";  
  String startTime      = "00:00:00"; // When we got to this location
  String endTime        = "00:00:00"; // When we last saw an event while here
  String counterMACAddresses[NUM_COUNTERS]    = {"",""};  // TODO: Consider moving into the "BaseStation"
  unsigned int counterEvents[NUM_COUNTERS][2] = {{0, 0},  // Counter 0, in/out count
                                                 {0, 0}}; // Counter 1, in/out count                              
};

ShuttleStop currentShuttleStop;

String shuttleRouteReport; // Aggreates the event summaries for each shuttle stop. 


// TODO: Read these from a configuration file
// "Basestation" wants to be an Object... Many of these might be his properties.
String routeName   = "Route 1";
String shuttleName = "Shuttle 6";

String reportingLocation = "AndroidAP3AE2";
String reportingLocationPassword = "ohpp8971";
String knownLocations[] = {"_Bighead", 
                           "Tower88", 
                           "William Shatner's Toupee", 
                           "Pretty Fly For A Wi-Fi V4",
                           "_4th_StreetPizzaCo", 
                           "SanPedroSquareMarket",
                           "AndroidAP3AE2"}; // Saving in PROGMEM 
                               
String counterNames[] = {"ShuttleCounter_c610", "ShuttleCounter_5ccc"}; 

                         
String currentLocation  = "Unknown"; 
String previousLocation = "Unknown";


// Multi-Tasking
SemaphoreHandle_t mutex_v;     // Mutex used to protect variables across RTOS tasks. 
TaskHandle_t eInkManagerTask;  // A task to update the eInk display.

// Flags for Tasks to take action
bool displayUpdateNeeded   = false;


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
void saveParameters();

// Bluetooth Counters
bool connectToCounter(BluetoothSerial &btUART, String counter);
void reconnectToCounter(BluetoothSerial &btUART);
void processMessage(BluetoothSerial &btUART, int counterID);
String sendReceive(BluetoothSerial &btUART, String stringToSend, int counterID);


// ShuttleStop wants to be a class...
void   resetShuttleStop(ShuttleStop &shuttleStop);
void   updateShuttleStop(ShuttleStop &shuttleStop, String jsonMessage, int counterNumber);
void   updateShuttleStop2(ShuttleStop &shuttleStop, String jsonMessage, int counterNumber);
String getShuttleStopJSON(ShuttleStop &shuttleStop);

// RouteReport wants to be a class...
void   resetRouteReport(String &routeReport);
void   appendRouteReport(String &routeReport, ShuttleStop &shuttleStop);
String formatRouteReport(String &routeReport);


//****************************************************************************************
// SETUP - Device initialization                                   
//****************************************************************************************
void setup(){
  Serial.begin(115200);   // Intialize terminal serial port
  delay(1000);            // Give port time to initalize

  mutex_v = xSemaphoreCreateMutex(); // The mutex we will use to protect variables 
                                     // across tasks
  showSplashScreen();
  loadParameters();
  configureIO();
  configureDisplay();
  configureRTC();
  networkConfig.ssid = reportingLocation;
  networkConfig.password = reportingLocationPassword;
  configureWiFi();
  configureBluetooth();
   
  // TODO: add a second counter when I build one...
  
  //connectToCounter(btUART1, counterNames[SHUTTLE]);
  sendReceive(btUART1, "-",0);
  sendReceive(btUART1, "g",0);
  sendReceive(btUART1, "c",0);
  
  //connectToCounter(btUART2, counterNames[TRAILER]);
  sendReceive(btUART2, "-",1);
  sendReceive(btUART2, "g",1);
  sendReceive(btUART2, "c",1);
  
  resetShuttleStop(currentShuttleStop);
  resetRouteReport(shuttleRouteReport);

  configureEinkManagerTask();
  
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("RUNNING!");
  xSemaphoreTake(mutex_v, portMAX_DELAY); 
    DEBUG_PRINT("mainLoop Running on Core #: ");
    DEBUG_PRINTLN(xPortGetCoreID());
  xSemaphoreGive(mutex_v);
  
  DEBUG_PRINTLN();
}

void deliverRouteReport(String shuttleRouteReport){
  
  String reportToIssue = shuttleRouteReport; 

  if (WiFi.status() != WL_CONNECTED){  
    enableWiFi(networkConfig);
  }
  if (WiFi.status() == WL_CONNECTED){
    postJSON(reportToIssue, networkConfig);  
  } 
}

void processLocationChange(){

  // Log what happened at the last location
  if (currentShuttleStop.endTime != "00:00:00"){ // Don't append if no events are present. 
    appendRouteReport(shuttleRouteReport, currentShuttleStop);
  }
  
  // Get ready to take data here, at the new location
  resetShuttleStop(currentShuttleStop);
  
  if (currentLocation == reportingLocation){
    DEBUG_PRINTLN("We have arrived at the reportingLocation!"); 
    DEBUG_PRINTLN("ROUTE REPORT: ");
    DEBUG_PRINTLN(shuttleRouteReport);

    // The WiFi Reporting task Touches the Shuttle Route Report as well.
    xSemaphoreTake(mutex_v, portMAX_DELAY); 
      // Tidy up the routeReport for delivery
      shuttleRouteReport = formatRouteReport(shuttleRouteReport);
    xSemaphoreGive(mutex_v);
    
    deliverRouteReport(shuttleRouteReport);
    
    // Get ready to start again
    xSemaphoreTake(mutex_v, portMAX_DELAY); 
      resetRouteReport(shuttleRouteReport);
    xSemaphoreGive(mutex_v);
    
  }else if (previousLocation == reportingLocation){
    DEBUG_PRINTLN("We have left the reportingLocation!");
  } else {
    DEBUG_PRINTLN("We have changed shuttle stops!");
  }
  
  previousLocation = currentLocation;
  //xSemaphoreTake(mutex_v, portMAX_DELAY); 
     displayUpdateNeeded = true;
  //xSemaphoreGive(mutex_v);
}


//****************************************************************************************
// Main Loop                                   
//****************************************************************************************
unsigned int t1=0,t2=t1;
unsigned int t3=0,t4=t3;

void loop(){

  t2=millis();
  if ((t2-t1)>60000){
    currentLocation  = scanForKnownLocations(knownLocations, NUMITEMS(knownLocations));
    DEBUG_PRINTLN("Previous Location: " + previousLocation + " Current Location: " + currentLocation);
    t1=millis(); 
  }

  if (currentLocation != previousLocation){ // We've moved

    DEBUG_PRINTLN("New Location!");
    String counterStats = sendReceive(btUART1, "g",0); // Get the last state
    updateShuttleStop2(currentShuttleStop, counterStats, 0);
    counterStats = sendReceive(btUART1, "c",0); // Clear the counter

    counterStats = sendReceive(btUART2, "g",1); // Get the last state
    updateShuttleStop2(currentShuttleStop, counterStats, 1);
    counterStats = sendReceive(btUART2, "c",1); // Clear the counter

    
    processLocationChange();
  }

  // We have just received some data. 
  // Take the message from the counter and update the current shuttleStop struct.
  xSemaphoreTake(mutex_v, portMAX_DELAY); 
    //if (btUART2.available()) {
    //  processMessage(btUART2, 1); 
    //}
    if (btUART1.available()) {
      
      processMessage(btUART1, 0); 
    }
  xSemaphoreGive(mutex_v);

  t4=millis();
  if ((t4-t3)>5000){
    String counterStats = sendReceive(btUART1, "g", 0);
    updateShuttleStop2(currentShuttleStop, counterStats, 0);

    counterStats = sendReceive(btUART2, "g", 1);
    updateShuttleStop2(currentShuttleStop, counterStats, 1);

    
    t3=millis();
  }    
  
  // Check for lost connection. Try and reconnect if lost.
  /*
  if (!btUART1.connected()){
    reconnectToCounter(btUART1);
  }
  if (!btUART2.connected()){
    reconnectToCounter(btUART2);
  }
  */
  
  delay(20);
}


// Tasks:

//****************************************************************************************
// A TASK that runs on Core0. Updates the eInk display with the currentShuttleStop data
// if it has changed.
//****************************************************************************************
void eInkManager(void *parameter){
  const unsigned int updateInterval = 5000;
  for(;;){ 
    vTaskDelay(updateInterval / portTICK_PERIOD_MS);
    if (displayUpdateNeeded){
      showCountDisplay(currentShuttleStop);
      //xSemaphoreTake(mutex_v, portMAX_DELAY); 
         displayUpdateNeeded = false;
      //xSemaphoreGive(mutex_v);
    }
  }
}

// IO Routines 
void loadParameters(){
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


void saveParameters(){
  
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
  n = sprintf(stopInfo, "Shuttle:\n       In:  %d\n       Out: %d\n\nTrailer:\n       In:  %d\n       Out: %d\n",
              shuttleStop.counterEvents[SHUTTLE][INBOUND],
              shuttleStop.counterEvents[SHUTTLE][OUTBOUND],
              shuttleStop.counterEvents[TRAILER][INBOUND],
              shuttleStop.counterEvents[TRAILER][OUTBOUND]);

  if (shuttleStop.location == reportingLocation){
    displayTextScreen("Visitors' Center", stopInfo);
  } else {
    displayTextScreen(shuttleStop.location.c_str(), stopInfo);
  }             
}

// WiFI 

//****************************************************************************************
// Scans to see if we are at a known location. This version uses WiFi networks. 
// Here, we return the strongest known SSID we can see. If we don't see any, we return
// "Unknown"
//****************************************************************************************
String scanForKnownLocations(String knownLocations[], int arraySize){
  String retValue = "Unknown_" + String(getRTCHour()) +":" + String(getRTCMinute()) +":" + String(getRTCSecond());
  //DEBUG_PRINTLN(" Scanning...");
  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  //DEBUG_PRINT("Free Heap: ");
  //DEBUG_PRINTLN(ESP.getFreeHeap());
  //DEBUG_PRINTLN(" Done.");

  if (n == 0) {
    DEBUG_PRINTLN(" No networks found.");
  } else {
    for (int i = 0; i < n; ++i) {
      // Return the first match to our known SSID list -- Return values are ordered by RSSI
      for (int j = 0; (j<arraySize); j++){
        if (WiFi.SSID(i) == knownLocations[j].c_str()){
          retValue = knownLocations[j];
          return retValue;
        }
      }       
    }
  }
  return retValue;
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
  centerPrint("Passenger", 70);
  centerPrint("Counting System", 90);
  centerPrint("Version", 110);
  centerPrint(SW_VERSION, 130);
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
  //btUART2.begin("ShuttleBasestationB", true); // Bluetooth device name 
                                            //  TODO: Provide opportunity to change names. 
  //delay(500); // Give port time to initalize
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

void connectToCounterAddress(BluetoothSerial &btUART, uint8_t remoteAddress[]){
// Comment from the library author: 
// connect(address) is fast (upto 10 secs max), connect(name) is slow (upto 30 secs max) as it needs
// to resolve name to address first, but it allows to connect to different devices with the same name.
// Set CoreDebugLevel to Info to view devices bluetooth address and device names
  
  DEBUG_PRINT("  Connecting to address...");
  bool connected = btUART.connect(remoteAddress);
  
  if(connected) {
    DEBUG_PRINTLN(" Success!");
  } else {
    while(!btUART.connected(10000)) {
      DEBUG_PRINTLN(" Failed to connect. Make sure remote device is available and in range, then restart app."); 
    }
  }  

  
}

bool connectToCounter(BluetoothSerial &btUART, String counter){ 
// Comment from the library author: 
// connect(address) is fast (upto 10 secs max), connect(name) is slow (upto 30 secs max) as it needs
// to resolve name to address first, but it allows to connect to different devices with the same name.
// Set CoreDebugLevel to Info to view devices bluetooth address and device names
  
  DEBUG_PRINT("  Connecting to " + counter + "...");
  bool connected = btUART.connect(counter);
  
  if(connected) {
    DEBUG_PRINTLN(" Success!");
  } else {
    while(!btUART.connected(10000)) {
      DEBUG_PRINTLN(" Failed to connect. Make sure remote device is available and in range, then restart app."); 
    }
  } 

  return connected; 
}


String sendReceive(BluetoothSerial &btUART, String stringToSend, int counterID){
  
  String inString="";

  if (connectToCounter(btUART, counterNames[counterID])){
  
    btUART.println(stringToSend);
    while (!(btUART.available())){
      delay(10);
    }  
  
    if ( btUART.available() ) inString = btUART.readStringUntil('\n');
    inString.trim();
    DEBUG_PRINTLN("Reply to:  " + stringToSend);
    DEBUG_PRINTLN(inString);

  }

  btUART.disconnect();
  
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
  deserializeJson(counterMessage, jsonMessage);

  shuttleStop.endTime = getRTCTime();

  shuttleStop.counterMACAddresses[counterNumber]= (const char *)counterMessage["deviceMAC"];
   
  if (counterMessage["eventType"] =="inbound"){
    shuttleStop.counterEvents[counterNumber][INBOUND]++;   //  = (const char *)counterMessage["count"];  
  }else{
    shuttleStop.counterEvents[counterNumber][OUTBOUND]++;  // = (const char *)counterMessage["count"];  
  }

  /*counterMessage["eventTime"] = getRTCTime();
  String test;
  serializeJson(counterMessage, test);
  DEBUG_PRINTLN(test);
  appendFile(SPIFFS,"/events.txt",test+",");
  DEBUG_PRINTLN(readFile(SPIFFS,"/events.txt"));
  */
}

//****************************************************************************************
// Update the shuttleStop with data from one of the counters. 'message' is the JSON 
// string the counters send us over Bluetooth.                                   
//****************************************************************************************
void updateShuttleStop2(ShuttleStop &shuttleStop, String jsonMessage, int counterNumber){
  StaticJsonDocument<2048> counterMessage;  
  deserializeJson(counterMessage, jsonMessage);

  unsigned int inCount = atoi((const char *)counterMessage["inbound"]);
  unsigned int outCount = atoi((const char *)counterMessage["outbound"]);
  

  if ((inCount  != shuttleStop.counterEvents[counterNumber][INBOUND]) || 
      (outCount != shuttleStop.counterEvents[counterNumber][OUTBOUND])) 
  
  {
    shuttleStop.endTime = getRTCTime();
    shuttleStop.counterMACAddresses[counterNumber]= (const char *)counterMessage["deviceMAC"];
    shuttleStop.counterEvents[counterNumber][INBOUND] = inCount;
    shuttleStop.counterEvents[counterNumber][OUTBOUND] = outCount;

    displayUpdateNeeded = true;
    
  }
}


//****************************************************************************************
// Dump the base station struct in JSON                                  
//****************************************************************************************
String getShuttleStopJSON(ShuttleStop &shuttleStop){   
  StaticJsonDocument<2048> shuttleStopJSON;
  
  shuttleStopJSON["location"]               = shuttleStop.location;
  shuttleStopJSON["startTime"]              = shuttleStop.startTime;
  shuttleStopJSON["endTime"]                = shuttleStop.endTime;
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
void resetRouteReport(String &routeReport){
    routeReport = "{\"routeName\":\"" + routeName    + "\"," +
                   "\"shuttleName\":\"" + shuttleName    + "\"," + 
                   "\"macAddress\":\"" + getMACAddress() + "\"," +
                   "\"shuttleStops\":[";  

}


//****************************************************************************************
// Body of the JSON shuttle route report.                                 
//****************************************************************************************
void appendRouteReport(String &routeReport, ShuttleStop &shuttleStop){
  String shuttleStopJSON = getShuttleStopJSON(shuttleStop);
  routeReport = routeReport + shuttleStopJSON + ",";
}


//****************************************************************************************
// Tidy up the JSON routeReport.                               
//****************************************************************************************
String formatRouteReport(String &routeReport){  
  // The last entry will have a trailing comma. Remove it. 
  int pos = routeReport.lastIndexOf(',');
  if (pos >=0) routeReport.remove(pos);  
  // Close out the JSON payload 
  return routeReport + "]}";
}


//****************************************************************************************
// Grab a JSON message from a counter and use it to update the currentShuttleStop.                             
//****************************************************************************************
void processMessage(BluetoothSerial &btUART, int counterID){
  String inString = btUART.readStringUntil('\n');
  Serial.print("Incoming Message: ");
  Serial.println(inString);  
  updateShuttleStop(currentShuttleStop, inString, counterID);
  //xSemaphoreTake(mutex_v, portMAX_DELAY); 
     displayUpdateNeeded = true;
  //xSemaphoreGive(mutex_v);
}


//****************************************************************************************
// If we loose the Bluetooth counter connection, try to get it back.                               
//****************************************************************************************
void reconnectToCounter(BluetoothSerial &btUART){
  Serial.println("Reconnecting...");
  btUART.connect(); // Uses previous address / name  
}
