/*
    Shuttle_Basestation
    
    An application to log data from directional people-counters on shuttle buses and report
    rider In/Our behavior for the entire shuttle route to the Parkdata System.

    The application attempts to use local WiFi SSIDs to determine the bus' location at 
    the various shuttle stops.
     
    Data will be buffered until the bus gets within range of a particular SSID where 
    aggregated data from all the stops gets reported.
    
    Written for the ESP32 WROOM Dev board V4 (Incl. WiFi, Bluetooth, and stacks of I/O.)
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/hw-reference/esp32c3/user-guide-devkitc-02.html

    Copyright 2021, Digame Systems. All rights reserved.
*/
#define SW_VERSION "1.0.0"
#define INBOUND  0  // The direction of counter events
#define OUTBOUND 1
#define COUNTER_0 0 // Counter indicies in data structures
#define COUNTER_1 1
#define SHUTTLE COUNTER_0 // Friendlier names for the indicies
#define TRAILER COUNTER_1


#include <digameDebug.h>     // Serial debugging defines. 
#include <digameTime.h>      // Time functions for RTC, NTP, etc. 
#include <digameNetwork_v2.h>   // For connections, MAC Address and reporting.
#include <digameDisplay.h>   // eInk Display support.
#include "BluetoothSerial.h" // Virtual UART support for Bluetooth Classic 
#include <WiFi.h>           
#include <ArduinoJson.h>     

BluetoothSerial btUART; // Create a BlueTooth Serial port to talk to the counter

NetworkConfig networkConfig;

struct ShuttleStop { 
  String location       = "Unknown";  // The SSID of the strongest known base station we can see.
  String startTime      = "00:00:00"; // When we got there
  String endTime        = "00:00:00"; // When we saw the last event while here
  
  String counterMACAddresses[2]    = {"",""};
  unsigned int counterEvents[2][2] = {{0, 0},  // Counter 0, in/out
                                      {0, 0}}; // Counter 1, in/out                               
};

ShuttleStop currentShuttleStop;
String shuttleRouteReport; // Aggreates the event summaries for each shuttle stop. 

// TODO: Read these from a configuration file
String shuttleName = "Shuttle 6";

String reportingLocation = "AndroidAP3AE2";//"Bighead"; ////"William Shatner's Toupee";

String knownLocations[] PROGMEM = {"_Bighead", 
                                   "Tower88", 
                                   "William Shatner's Toupee", 
                                   "Pretty Fly For A Wi-Fi V4",
                                   "4th_StreetPizzaCo", 
                                   "SanPedroSquareMarket",
                                   "AndroidAP3AE2"}; // Saving in PROGMEM 
                               
String counterNames[] = {"ShuttleCounter_5ccc",
                         "ShuttleCounter_aaaa"};

String currentLocation  = "Unknown"; // Updated in locationScanner task. (Read-only everywhere else!)
String previousLocation = "Unknown";

TaskHandle_t locationScannerTask;  // A task to look for known networks. Runs on Core 0.
bool messageDeliveryNeeded = false;


// Utility Function: Number of items in an array
#define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

//****************************************************************************************
// Declares                                  
//****************************************************************************************

// WiFi
String scanForKnownLocations(String knownLocations[], int arraySize);
void locationScanner(void *parameter); // TASK that runs on Core 0

// UI
void showMenu();
void showSplashScreen();
void showCountDisplay(ShuttleStop &shuttleStop);

// Initialization
void configureIO();
void configureDisplay();
void configureRTC();
void configureWiFi();
void configureBluetooth(); 
void configureWiFiScanTask();

// Bluetooth Counters
void connectToCounter(BluetoothSerial &btUART, String counter);
void reconnectToCounter(BluetoothSerial &btUART);
void processMessage(BluetoothSerial &btUART, int counterID);

// Data Mgt.
void resetShuttleStop(ShuttleStop &shuttleStop);
void updateShuttleStop(ShuttleStop &shuttleStop, String jsonMessage, int counterNumber);
String getShuttleStopJSON(ShuttleStop &shuttleStop);

// Route Report wants to be a class...
void resetRouteReport(String &routeReport);
void appendRouteReport(String &routeReport, ShuttleStop &shuttleStop);
String formatRouteReport(String &routeReport);
void deliverRouteReport(String &routeReport);



//****************************************************************************************
// SETUP - Device initialization                                   
//****************************************************************************************
void setup(){

  Serial.begin(115200);   // Intialize terminal serial port
  delay(1000);            // Give port time to initalize

  showSplashScreen();
  configureIO();
  configureDisplay();
  configureRTC();
  configureWiFi();
  configureBluetooth();
  configureLocationScanTask();
  
  // TODO: add a second counter when I build one...
  connectToCounter(btUART, counterNames[COUNTER_0]);
  
  resetShuttleStop(currentShuttleStop);
  resetRouteReport(shuttleRouteReport);
  
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("RUNNING!");
  DEBUG_PRINTLN();
   
}

//****************************************************************************************
// Main Loop                                   
//****************************************************************************************
void loop(){

  if (currentLocation != previousLocation){ // we've moved between SSIDs
    //TODO: append report structure...
    //
    appendRouteReport(shuttleRouteReport, currentShuttleStop);
    resetShuttleStop(currentShuttleStop);
    
    if (currentLocation == reportingLocation){
      DEBUG_PRINTLN("We have arrived at the reportingLocation!"); 
      DEBUG_PRINTLN("ROUTE REPORT: ");
      DEBUG_PRINTLN(shuttleRouteReport);
      shuttleRouteReport = formatRouteReport(shuttleRouteReport);
      deliverRouteReport(shuttleRouteReport);
      resetRouteReport(shuttleRouteReport);
      
    }else if (previousLocation == reportingLocation){
      DEBUG_PRINTLN("We have left the reportingLocation!");
    } else {
      DEBUG_PRINTLN("We have changed shuttle stops!");
    }
  
    previousLocation = currentLocation;
  }

  // We have just received some data. 
  // Take the message from the counter and update the current shuttleStop struct.
  
  
  if (btUART.available()) {
    processMessage(btUART, 0); 
  }
  
  // Check for lost connection.
  if (!btUART.connected()){
    reconnectToCounter(btUART);
  }

  delay(20);
  
}

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


void showCountDisplay(ShuttleStop &shuttleStop){
  
  char stopInfo [150];
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

//****************************************************************************************
// Scans to see if we are at a known location. This version uses WiFi networks. 
// Here, we return the strongest known SSID we can see. If we don't see any, we return
// "Unknown"
//****************************************************************************************
String scanForKnownLocations(String knownLocations[], int arraySize){
  String retValue = "Unknown";
  
  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  if (n == 0) {
    DEBUG_PRINTLN(" No networks found.");
  } else {
    //DEBUG_PRINT(n);
    //DEBUG_PRINTLN(" Networks found...");
    for (int i = 0; i < n; ++i) {
      //DEBUG_PRINT(WiFi.SSID(i) + " ");
      //DEBUG_PRINTLN(WiFi.RSSI(i));
      // Return the first match to our known SSID list -- Return values are ordered by RSSI
      for (int j = 0; (j<arraySize); j++){
        if (WiFi.SSID(i) == knownLocations[j].c_str()){
          //DEBUG_PRINT("Match Found: ");
          //DEBUG_PRINTLN(WiFi.SSID(i));
          retValue = knownLocations[j];
          return retValue;
        }
      }       
    }
  }
  return retValue;
}

//****************************************************************************************
// A Task that runs on Core0 to to determine our current location.
//****************************************************************************************
void locationScanner(void *parameter){
  const unsigned int countDownTimeout = 5000; 
  static unsigned int scanCountDown = countDownTimeout;
  for(;;){  
    scanCountDown = scanCountDown - 100;
    vTaskDelay(100 / portTICK_PERIOD_MS);

    if (messageDeliveryNeeded){    
      if (WiFi.status() != WL_CONNECTED){  
        enableWiFi(networkConfig);
      }
      if (WiFi.status() == WL_CONNECTED){
        postJSON(shuttleRouteReport, networkConfig);  
      }
      messageDeliveryNeeded = false;
    }
    
    if (scanCountDown <=0){
      currentLocation  = scanForKnownLocations(knownLocations, NUMITEMS(knownLocations));
      scanCountDown = countDownTimeout;
      DEBUG_PRINTLN("Previous Location: " + previousLocation + " Current Location: " +currentLocation);
    }   
  }
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
  DEBUG_PRINTLN("*******************");
  DEBUG_PRINTLN("TODO: Init Display.");
  DEBUG_PRINTLN("*******************");

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
  btUART.begin("ShuttleBasestation", true); // Bluetooth device name 
                                        //  TODO: Provide opportunity to change names. 
  delay(1000);                          // Give port time to initalize
}

void configureLocationScanTask(){
  // Set up a separate task to scan for WiFi networks to see if we are in a location 
  // we are familiar with. 
  // Task that will be executed in the locationScanner() function with priority 0 and 
  // executed on core 0.
  
  xTaskCreatePinnedToCore(
    locationScanner,      // Task function. 
   "Location Scanner",    // name of task. 
    10000,                // Stack size of task 
    NULL,                 // parameter of the task 
    0,                    // priority of the task 
    &locationScannerTask, // Task handle to keep track of created task 
    0);                   // pin task to core 0  
      
}

void connectToCounter(BluetoothSerial &btUART, String counter){
  
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
}

//****************************************************************************************
// Initialize the shuttleStop structure. Called when we leave a stop.                                    
//****************************************************************************************
void resetShuttleStop(ShuttleStop &shuttleStop){
  shuttleStop.location = currentLocation;
  shuttleStop.startTime = getRTCTime();
  shuttleStop.endTime = shuttleStop.startTime;
  shuttleStop.counterEvents[COUNTER_0][INBOUND] = 0;
  shuttleStop.counterEvents[COUNTER_0][OUTBOUND] = 0;
  shuttleStop.counterEvents[COUNTER_1][INBOUND] = 0;
  shuttleStop.counterEvents[COUNTER_1][OUTBOUND] = 0;
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
}

//****************************************************************************************
// Dump the base station struct in JSON                                  
//****************************************************************************************
String getShuttleStopJSON(ShuttleStop &shuttleStop){   
  StaticJsonDocument<2048> shuttleStopJSON;
  
  shuttleStopJSON["location"]               = shuttleStop.location;
  shuttleStopJSON["startTime"]              = shuttleStop.startTime;
  shuttleStopJSON["endTime"]                = shuttleStop.endTime;
  shuttleStopJSON["shuttle"]["macAddress"]  = shuttleStop.counterMACAddresses[COUNTER_0];
  shuttleStopJSON["shuttle"]["inbound"]  = shuttleStop.counterEvents[COUNTER_0][INBOUND];
  shuttleStopJSON["shuttle"]["outbound"] = shuttleStop.counterEvents[COUNTER_0][OUTBOUND];
  shuttleStopJSON["trailer"]["macAddress"]  = shuttleStop.counterMACAddresses[COUNTER_1];
  shuttleStopJSON["trailer"]["inbound"]  = shuttleStop.counterEvents[COUNTER_1][INBOUND];
  shuttleStopJSON["trailer"]["outbound"] = shuttleStop.counterEvents[COUNTER_1][OUTBOUND];

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
  routeReport = "{\"shuttleName\":\"" + shuttleName    + "\"," + 
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
// Suffix and Value of the JSON route report.                               
//****************************************************************************************
String formatRouteReport(String &routeReport){  
  // last entry will have a trailing comma. Remove it. 
  int pos = routeReport.lastIndexOf(',');
  if (pos >=0) routeReport.remove(pos);
  return routeReport + "]}";
}


void deliverRouteReport(String &routeReport){
  DEBUG_PRINTLN("*********************");
  DEBUG_PRINTLN("TODO: POST to server.");
  DEBUG_PRINTLN("*********************");

  messageDeliveryNeeded = true; 
  
  while (messageDeliveryNeeded){
    delay(100);  
  }
 
  return; 
}


void updateDisplay(ShuttleStop &shuttleStop){
  DEBUG_PRINTLN("**************************");
  DEBUG_PRINTLN("TODO: eInk Display...");
  DEBUG_PRINTLN("Location: " + shuttleStop.location);
  DEBUG_PRINTLN(" Shuttle:");
  DEBUG_PRINTLN("  IN:  " + String(shuttleStop.counterEvents[SHUTTLE][INBOUND])); 
  DEBUG_PRINTLN("  OUT: " + String(shuttleStop.counterEvents[SHUTTLE][OUTBOUND])); 
  DEBUG_PRINTLN(" Trailer:");
  DEBUG_PRINTLN("  IN:  " + String(shuttleStop.counterEvents[TRAILER][INBOUND])); 
  DEBUG_PRINTLN("  OUT: " + String(shuttleStop.counterEvents[TRAILER][OUTBOUND])); 
  DEBUG_PRINTLN("**************************");
  
  DEBUG_PRINTLN(); 

  showCountDisplay(shuttleStop);
  
}

//****************************************************************************************
// Grab a JSON message from a counter and use it to update the currentShuttleStop.                             
//****************************************************************************************
void processMessage(BluetoothSerial &btUART, int counterID){
  String inString = btUART.readStringUntil('\n');

  Serial.print("Incoming Message: ");
  Serial.println(inString);  

  updateShuttleStop(currentShuttleStop, inString, counterID);
  updateDisplay(currentShuttleStop);
    
}


//****************************************************************************************
// If we loose the Bluetooth counter connection, try to get it back.                               
//****************************************************************************************
void reconnectToCounter(BluetoothSerial &btUART){
  Serial.println("Reconnecting...");
  btUART.connect(); // Uses previous address / name  
}
