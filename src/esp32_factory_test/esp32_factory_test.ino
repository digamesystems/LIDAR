/*
    Heimdall -- "Heimdall sees all!" 

    This program uses a pair of TFMini-plus LIDARs to count people boarding /
    unboarding shuttle buses. 
    
    The two sensors are used in combination to determine direction of travel and 
    thereby if a person is boarding or exiting.
    
    Data is reported in JSON format via Bluetooth classic. 
    
    LIDAR Sensor:
    http://en.benewake.com/product/detail/5c345cd0e5b3a844c472329b.html
    (See manual in /docs folder.)

    Written for the ESP32 WROOM Dev board V4 (Incl. WiFi, Bluetooth, and stacks of I/O.)
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/hw-reference/esp32c3/user-guide-devkitc-02.html

    Copyright 2021, Digame Systems. All rights reserved.

    Testing the Sensor: 
    https://youtu.be/LPxUawpHUEk
    https://youtu.be/7mM1T-jgChU
    https://youtu.be/dQw4w9WgXcQ

*/

#define HARDWARE_PRESENT true // Flag so we can debug the code without LIDAR sensors.

//****************************************************************************************
//****************************************************************************************

#define tfMiniUART_1 Serial1
#define tfMiniUART_2 Serial2


//****************************************************************************************
//****************************************************************************************
#include <digameDebug.h>     // Serial debugging defines. 
#include <digameFile.h>      // Read/Write Text files.
#include <digameNetwork.h>   // For MAC address functions

#include "BluetoothSerial.h" // Part of the ESP32 board package. 
                             // By Evandro Copercini - 2018

#include <TFMPlus.h>         // Include TFMini Plus LIDAR Library v1.4.0
                             // https://github.com/budryerson/TFMini-Plus

#include <SPIFFS.h>          // FLASH file system support.


// For Over the Air (OTA) updates... 
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

bool useOTA = true; 

//****************************************************************************************
//****************************************************************************************                        
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

TFMPlus tfmP_1;         // Create a TFMini Plus object for sensor 1
TFMPlus tfmP_2;         // Create a TFMini Plus object for sensor 2

BluetoothSerial btUART; // Create a BlueTooth Serial Port Object


//****************************************************************************************
//****************************************************************************************
// Smoothing the LIDAR signals to get rid of noise and introduce a bit of a decay time. 
float smoothed_LIDAR_1 = 0.0;
float smoothed_LIDAR_2 = 0.0;

// We're tracking the visibility of the target on both sensors. Valid events go from only 
// visible on one sensor to being visible on both. Direction is determined by which sensor
// sees the target first. 
int previousState = 0; 
int state         = 0; 

#define OUTBOUND  1
#define INBOUND   2
#define BOTH      3

unsigned int inCount  = 0;
unsigned int outCount = 0;

String deviceName        = "Shuttle 6";
float  distanceThreshold = 160;
float  smoothingFactor   = 0.95;

bool streamingRawData = false; 
bool menuActive = false;

bool clearDataFlag = false; 

String jsonPayload;
String jsonPrefix;  


//****************************************************************************************
// Overloading print and println to send messages to both serial and bluetooth connections
// There must be a better way to do this with a #define 
//****************************************************************************************
void dualPrintln(String s="");
void dualPrint(String s="");
void dualPrintln(float f);
void dualPrint(float f);
void dualPrintln(int i);
void dualPrint(int i);

void loadDefaults();

void showSplashScreen();
void showMenu();

String getUserInput();
void   scanForUserInput();

void  configureWiFi();
void  configureBluetooth();
void  configureLIDARs();

void  configureOTA();

void initLIDAR(TFMPlus &tfmP, int port=1);
int  processLIDAR(TFMPlus &tfmP, float &smoothed, int offset);


//****************************************************************************************                            
void setup() // - Device initialization
//****************************************************************************************
{
  Serial.begin(115200);   // Intialize terminal serial port
  delay(1000);            // Give port time to initalize
  
  loadDefaults();
  
  DEBUG_PRINTLN("INITIALIZING HARDWARE...");
  
  configureWiFi();
  configureBluetooth();
  configureLIDARs();

  if (useOTA) {
    configureOTA();  
  }

  // The first part of all of our JSON messages
  jsonPrefix = "{\"deviceName\":\"" + deviceName + "\",\"deviceMAC\":\"" + WiFi.macAddress();

  DEBUG_PRINTLN();
  DEBUG_PRINTLN("RUNNING!");

     
}


//****************************************************************************************
void loop()  // Main 
//****************************************************************************************
{ 
  scanForUserInput();
  
  if (clearDataFlag){
    inCount = 0; 
    outCount = 0;
    clearDataFlag = false;    
  }

  // Run a little state machine based on the visibility of a target
  // on the two LIDAR sensors

  int visible = 0;
  state = 0;

  // Target visible on sensor 1?
  visible = processLIDAR(tfmP_1, smoothed_LIDAR_1, 100); 
  
  if (visible>=0){
    
    state = state + visible;

    // Target visible on sensor 2?
    visible = processLIDAR(tfmP_2, smoothed_LIDAR_2, 110);
    state = state + visible * 2;

    if (streamingRawData) dualPrintln(distanceThreshold);

    if (state == BOTH){ // Visible on both Sensors
      jsonPayload = jsonPrefix; 
    
      if ((previousState == INBOUND)){ // INBOUND event
        inCount += 1;
        jsonPayload = jsonPayload + "\",\"eventType\":\"inbound" +
                       "\",\"count\":\"" + inCount + "\"" +
                       "}";
  
        if ((!streamingRawData)&&(menuActive)) dualPrintln(jsonPayload); 
      }
    
      if ((previousState == OUTBOUND)){ // OUTBOUND event
        outCount += 1;
        jsonPayload = jsonPayload + "\",\"eventType\":\"outbound" +
                      "\",\"count\":\"" + outCount + "\"" +
                      "}";
  
        if ((!streamingRawData)&&(menuActive)) dualPrintln(jsonPayload);
      }
      
    }
    previousState = state;
  }
}


//****************************************************************************************
void configureWiFi(){
//****************************************************************************************
  DEBUG_PRINTLN(" WiFi...");

}


//****************************************************************************************
void configureBluetooth(){
//****************************************************************************************
  DEBUG_PRINTLN(" Bluetooth...");
  btUART.begin("ShuttleCounter_" + getShortMACAddress()); // My Bluetooth device name 
                                  //  TODO: Provide opportunity to change names. 
  delay(1000);                    // Give port time to initalize
    
}


//****************************************************************************************
void configureLIDARs(){
//**************************************************************************************** 
  DEBUG_PRINTLN("  Configuring LIDARS...");

  #if HARDWARE_PRESENT
    DEBUG_PRINTLN("  LIDAR 1...");
    tfMiniUART_1.begin(115200,SERIAL_8N1,25,33);  // Initialize TFMPLus device serial port.
    delay(1000);                    // Give port time to initalize
    initLIDAR(tfmP_1, 1);
     
    DEBUG_PRINTLN("  LIDAR 2...");
    tfMiniUART_2.begin(115200,SERIAL_8N1,27,26);  // Initialize TFMPLus device serial port.
    delay(1000);
    initLIDAR(tfmP_2, 2);
  #endif

}


//***************************************************************************************
String processor(const String& var)
//***************************************************************************************
{
  if(var == "config.deviceName") return deviceName;
  return "";
}

//****************************************************************************************
void configureOTA(){
//****************************************************************************************

  DEBUG_PRINTLN("  Stand-Alone Mode. Setting AP (Access Point)…");  
  WiFi.mode(WIFI_AP);
  
  String netName = "ShuttleCounter_" + getShortMACAddress();
  const char* ssid = netName.c_str();
  WiFi.softAP(ssid);
  
  IPAddress IP = WiFi.softAPIP();
  DEBUG_PRINT("    AP IP address: ");
  DEBUG_PRINTLN(IP);   
  
  //delay(3000);  
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    if(!request->authenticate("admin", "admin"))
      return request->requestAuthentication();
      
    request->send(SPIFFS, "/index.html", String(), false, processor);
  
  });

  server.serveStatic("/", SPIFFS, "/"); // sets the base path for the web server
  AsyncElegantOTA.begin(&server);   
  server.begin();
}

  
//****************************************************************************************
// Simultaneous Print functions. Bluetooth and Serial. 
// TODO: find a better way to do this with #define macro... 
//****************************************************************************************
void dualPrintln(String s){
  DEBUG_PRINTLN(s);
  btUART.println(s);  
}

void dualPrint(String s){
  DEBUG_PRINT(s);
  btUART.print(s);  
}

void dualPrintln(float f){
  DEBUG_PRINTLN(f);
  btUART.println(f);  
}

void dualPrint(float f){
  DEBUG_PRINT(f);
  btUART.print(f);  
}

void dualPrintln(int i){
  DEBUG_PRINTLN(i);
  btUART.println(i);  
}

void dualPrint(int i){
  DEBUG_PRINT(i);
  btUART.print(i);  
}


//****************************************************************************************
void showSplashScreen(){
//****************************************************************************************
  String compileDate = F(__DATE__);
  String compileTime = F(__TIME__);

  dualPrintln();
  dualPrintln("*******************************************");
  dualPrintln("ParkData Directional LIDAR Sensor");
  dualPrintln("Version 1.0");
  dualPrintln("");
  dualPrintln("Compiled: " + compileDate + " at " + compileTime); 
  dualPrint("Device Name: ");
  dualPrintln(deviceName);
  dualPrint("Bluetooth Address: ShuttleCounter_");
  dualPrintln(String(getShortMACAddress()));
  dualPrintln();
  dualPrintln("Copyright 2022, Digame Systems.");
  dualPrintln("All rights reserved.");
  dualPrintln("*******************************************");
  dualPrintln();   
}


//****************************************************************************************
void showMenu(){
//****************************************************************************************
  if (menuActive){
  showSplashScreen();
  
  dualPrintln("MENU: ");
  dualPrintln("  [+][-]Menu Active");
  dualPrintln("  [N]ame               (" + deviceName +")");
  dualPrintln("  [D]istance threshold (" + String(distanceThreshold) + ")");
  dualPrintln("  [S]moothing factor   (" + String(smoothingFactor) + ")");
  dualPrintln("  [G]Get count data");
  dualPrintln("  [C]Clear count data");
  dualPrintln("  [R]aw Data Stream    (" + String(streamingRawData) + ")");
  dualPrintln();
  //dualPrintln("  Toggle [r]aw data stream ");
  }
}


//****************************************************************************************                            
void loadDefaults(){
//****************************************************************************************                            
  if(!SPIFFS.begin()){
    DEBUG_PRINTLN("    File System Mount Failed");
  } else {
    //DEBUG_PRINTLN("    SPIFFS up!");
    String temp;
    
    temp = readFile(SPIFFS, "/name.txt");
    if (temp.length() >0) deviceName  = temp;
    
    temp = readFile(SPIFFS, "/smooth.txt");
    if (temp.length() > 0) smoothingFactor = temp.toFloat();
    
    temp = readFile(SPIFFS, "/threshold.txt");
    if (temp.length() > 0) distanceThreshold = temp.toFloat();
  }
}


//****************************************************************************************
// Initialize a LIDAR sensor on a serial port. 
//****************************************************************************************
void initLIDAR(TFMPlus &tfmP, int port){
   
  // Initialize a TFminiPlus class and pass a serial port to the object.
  if (port == 1){
    tfmP.begin(&tfMiniUART_1);   
  }else if (port == 2){
    tfmP.begin(&tfMiniUART_2);   
  }else{
    DEBUG_PRINTLN("Unknown Port. I give up.");
    while (1){} // Nothing to do if we don't have a valid port.  
    return;
  }
     
  // Send some commands to configure the TFMini-Plus
  // Perform a system reset
  DEBUG_PRINT( "  Activating LIDAR Sensor... ");
  if( tfmP.sendCommand(SOFT_RESET, 0)){
      DEBUG_PRINTLN("Sensor Active.");
  }
  else{
    DEBUG_PRINTLN("   TROUBLE ACTIVATING LIDAR!");                    
    tfmP.printReply();
  }

  delay(1000);

  // Set the acquisition rate to 100 Hz.
  DEBUG_PRINT( "  Adjusting Frame Rate... ");
  if( tfmP.sendCommand(SET_FRAME_RATE, FRAME_100)){
      DEBUG_PRINTLN("Frame Rate Adjusted.");
  }
  else tfmP.printReply();
 
}

//****************************************************************************************
// Read the LIDAR sensor and determine if a target is present in the field. Uses a bit of 
// history in the form of an exponentially smoothed version of the raw signal.
//****************************************************************************************
int processLIDAR(TFMPlus &tfmP, float &smoothed, int offset){
  int16_t tfDist = 0;    // Distance to object in centimeters
  int16_t tfFlux = 0;    // Strength or quality of return signal
  int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
  
  int lidarUpdateRate = 5; // 100Hz -> 10 ms
  int targetVisible = false;

#if HARDWARE_PRESENT 
  //tfmP.sendCommand(TRIGGER_DETECTION, 0);

  //lightSleepMSec(lidarUpdateRate);
  delay(lidarUpdateRate);

  // Read the LIDAR Sensor
  if( tfmP.getData( tfDist, tfFlux, tfTemp)) { 

    //Filter the measured distance
    smoothed = smoothed * smoothingFactor + (float)tfDist * (1-smoothingFactor); 

    targetVisible = (smoothed < distanceThreshold);  

    if (streamingRawData) {  
      //dualPrint(tfDist + offset);
      //dualPrint(" ");
      dualPrint(smoothed);
      dualPrint(" ");
      if (targetVisible){
         dualPrint(300 + offset);
      } else {
        dualPrint(200 + offset);
      }
      dualPrint(" ");
    }

    if (targetVisible) {
      return 1;
    } else {
      return 0;
    }
   
  }

  return -1; 
#else // Emulating the presence of the LIDAR sensor
  delay(10);
  return random(2); // returns a random number from 0 to 1
#endif
}


//****************************************************************************************
String getUserInput() { // -- No range checking!
//****************************************************************************************
  String inString;

  while (!(debugUART.available()) && !(btUART.available())){
    delay(10);
    //lightSleepMSec(10);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
  }  

  if ( debugUART.available() ) inString = debugUART.readStringUntil('\n');
  if ( btUART.available() ) inString = btUART.readStringUntil('\n');
  
  inString.trim();
  dualPrint(" You entered: ");
  dualPrintln(inString);
  return inString;  

}


//****************************************************************************************
void scanForUserInput()
//****************************************************************************************
{
  String inString;
  bool inputReceived=false;
 
  inputReceived = false;
  
  if (debugUART.available()){
    inString = debugUART.readStringUntil('\n');
    inputReceived = true;
  }  

  if (btUART.available()){
    inString = btUART.readStringUntil('\n');
    inputReceived = true;
  }
  
  if (inputReceived) {  
    inString.trim();
    //dualPrintln(inString); 

    if (inString == "+") {
      dualPrintln("OK");
      menuActive = true;
    }

    if (inString == "-") {
      dualPrintln("OK");
      menuActive = false;
    }
    
    if (inString == "n") {
      dualPrintln(" Enter New Device Name. (" + deviceName +")");
      deviceName = getUserInput();
      dualPrint(" New Device Name: ");
      dualPrintln(deviceName);
      writeFile(SPIFFS, "/name.txt", deviceName.c_str());
    } 
    
    if(inString == "d"){
      dualPrintln(" Enter New Distance Threshold. (" + String(distanceThreshold) +")");
      distanceThreshold = getUserInput().toFloat();
      dualPrint(" New distanceThreshold: ");
      dualPrintln(distanceThreshold);
      writeFile(SPIFFS, "/threshold.txt", String(distanceThreshold).c_str());
    } 
    
    if(inString == "s"){
      dualPrintln(" Enter New Smoothing Factor. (" + String(smoothingFactor) + ")");
      smoothingFactor = getUserInput().toFloat();
      dualPrint(" New Smoothing Factor: ");
      dualPrintln(smoothingFactor);
      writeFile(SPIFFS, "/smooth.txt", String(smoothingFactor).c_str());
    }

    if(inString == "g"){
      String jsonPayload = jsonPrefix + "\",\"inbound\":\""  + inCount  + "\"" + 
                                          ",\"outbound\":\"" + outCount + "\"" +
                                          "}";
      dualPrintln(jsonPayload);
      //dualPrintln("OK");
    } 

    if(inString == "c"){
      dualPrintln("OK");
      clearDataFlag = true;
    }      

    if(inString == "r"){
      streamingRawData = (!streamingRawData);
    } 
    
    if (!streamingRawData) showMenu(); 
  }   
}
