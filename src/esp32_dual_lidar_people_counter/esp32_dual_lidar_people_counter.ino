/*
    Heimdall -- "Heimdall sees all!" 

    This program uses a pair of TFMini-plus LIDARs to count people boarding /
    unboarding shuttle busses. 
    
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

//****************************************************************************************
// Defines
//****************************************************************************************
#define tfMiniUART_1 Serial1
#define tfMiniUART_2 Serial2

//****************************************************************************************
// Includes
//****************************************************************************************
#include <digameDebug.h>     // Serial debugging defines. 
#include <digameFile.h>      // Read/Write Text files.

#include "BluetoothSerial.h" // Part of the ESP32 board package. 
                             // By Evandro Copercini - 2018
                        
#include <TFMPlus.h>         // Include TFMini Plus LIDAR Library v1.4.0
                             // https://github.com/budryerson/TFMini-Plus
                        
#include <digameNetwork.h>   // For MAC address functions

#include <SPIFFS.h> 

//****************************************************************************************
// Objects
//****************************************************************************************                        
TFMPlus tfmP_1;         // Create a TFMini Plus object for sensor 1
TFMPlus tfmP_2;         // Create a TFMini Plus object for sensor 2
BluetoothSerial btUART; // Create a BlueTooth Serial Port Object


//****************************************************************************************
// Globals
//****************************************************************************************
// Smoothing the LIDAR signals to get rid of noise and introduce a bit of a decay time. 

float smoothed_LIDAR_1 = 0.0;
float smoothed_LIDAR_2 = 0.0;

// We're tracking the visibility of the target on both sensors. Valid events go from only 
// visible on one sensor to being visible on both. Direction is determined by which sensor
// sees the target first. 

int previousState = 0; 
int state = 0; 
unsigned int inCount = 0;
unsigned int outCount = 0;
float distanceThreshold = 160;
float smoothingFactor = 0.95;
bool showRawData = false; 
bool clearDataFlag = false; 

String deviceName = "Front Door";

String jsonPayload; 

//****************************************************************************************
// Simultaneous Print functions. TODO: find a better way to do this with #define macro... 
//****************************************************************************************
void dualPrintln(String s=""){
  DEBUG_PRINTLN(s);
  btUART.println(s);  
}

void dualPrint(String s=""){
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
// Initialize a LIDAR sensor on a serial port. 
//****************************************************************************************
void init_TFMPlus(TFMPlus &tfmP, int port=1){
   
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
  Serial.printf( "  Activating LIDAR Sensor... ");
  if( tfmP.sendCommand(SYSTEM_RESET, 0)){
      DEBUG_PRINTLN("Sensor Active.");
  }
  else{
    DEBUG_PRINTLN("   TROUBLE ACTIVATING LIDAR!");                    
    tfmP.printReply();
  }

  delay(1000);

  // Set the acquisition rate to 100 Hz.
  Serial.printf( "  Adjusting Frame Rate... ");
  if( tfmP.sendCommand(SET_FRAME_RATE, FRAME_100)){
      DEBUG_PRINTLN("Frame Rate Adjusted.");
  }
  else tfmP.printReply();
 
}

//****************************************************************************************
// Read the LIDAR sensor and determine if a target is present in the field. Uses a bit of 
// history in the form of an exponentially smoothed version of the raw signal.
//****************************************************************************************
int process_LIDAR(TFMPlus &tfmP, float &smoothed, int offset){
  int16_t tfDist = 0;    // Distance to object in centimeters
  int16_t tfFlux = 0;    // Strength or quality of return signal
  int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
  
  int lidarUpdateRate = 10; // 100Hz -> 10 ms
  int targetVisible = false;
  
  tfmP.sendCommand(TRIGGER_DETECTION, 0);
  //delay(lidarUpdateRate);

  // Read the LIDAR Sensor
  if( tfmP.getData( tfDist, tfFlux, tfTemp)) { 

    //Filter the measured distance
    smoothed = smoothed * smoothingFactor + (float)tfDist * (1-smoothingFactor); 


    targetVisible = (smoothed < distanceThreshold);  

    if (showRawData) {  
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
    
}


void showSplashScreen(){
  String compileDate = F(__DATE__);
  String compileTime = F(__TIME__);

  dualPrintln();
  dualPrintln("*****************************************************");
  dualPrintln("ParkData Directional LIDAR Sensor");
  dualPrintln("Version 1.0");
  dualPrintln("Compiled: " + compileDate + " at " + compileTime); 
  dualPrintln("Copyright 2021, Digame Systems. All rights reserved.");
  dualPrint("Device Name: ");
  dualPrintln(deviceName);
  dualPrint("Bluetooth Address: ShuttleCounter_");
  dualPrintln(String(getShortMACAddress()));
  dualPrintln();
  dualPrintln("*****************************************************");
  dualPrintln();   
}

//****************************************************************************************
// A little menu routine so users can set paramters
//****************************************************************************************
void showMenu(){
  showSplashScreen();
  
  dualPrintln("MENU: ");
  dualPrintln("  [N]ame               (" + deviceName +")");
  dualPrintln("  [D]istance threshold (" + String(distanceThreshold) + ")");
  dualPrintln("  [S]moothing factor   (" + String(smoothingFactor) + ")");
  dualPrintln("  [C]Clear count data");
  dualPrintln("  [R]aw Data Stream    (" + String(showRawData) + ")");
  dualPrintln();
  //dualPrintln("  Toggle [r]aw data stream ");
}


//****************************************************************************************
// Grab input from the user. -- No range checking!
//****************************************************************************************
String getUserInput(){
  String inString;

  while (!(debugUART.available()) && !(btUART.available())){
    delay(10);
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
// SETUP - Device initialization                                   
//****************************************************************************************
void setup()
{

  Serial.begin(115200);   // Intialize terminal serial port
  delay(1000);            // Give port time to initalize

  //dualPrintln("  Initializing SPIFFS...");

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
  
  showSplashScreen();

  DEBUG_PRINTLN("INITIALIZING HARDWARE");
  DEBUG_PRINTLN();
  DEBUG_PRINTLN(" WiFi...");
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  DEBUG_PRINTLN(" Bluetooth...");
  btUART.begin("ShuttleCounter_" + getShortMACAddress()); // Bluetooth device name 
                                  //  TODO: Provide opportunity to change names. 
  delay(1000);                    // Give port time to initalize
  
  DEBUG_PRINTLN(" LIDAR 1...");
  tfMiniUART_1.begin(115200,SERIAL_8N1,25,33);  // Initialize TFMPLus device serial port.
  delay(1000);                    // Give port time to initalize
  init_TFMPlus(tfmP_1, 1);
   
  DEBUG_PRINTLN(" LIDAR 2...");
  tfMiniUART_2.begin(115200,SERIAL_8N1,27,26);  // Initialize TFMPLus device serial port.
  delay(1000);
  init_TFMPlus(tfmP_2, 2);

  DEBUG_PRINTLN();
  DEBUG_PRINTLN("RUNNING!");
  DEBUG_PRINTLN();
     
}

//****************************************************************************************
void checkForUserInput(){
//****************************************************************************************

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
    dualPrintln(inString); 
    
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

    if(inString == "c"){
      dualPrint(" Data flagged for clear.");
      clearDataFlag = true;
    }      

    if(inString == "r"){
      showRawData = (!showRawData);
    } 
    
    if (!showRawData) showMenu(); 
  }   
}


//****************************************************************************************
// LOOP - Main Loop                                   
//****************************************************************************************
void loop(){
  state = 0;

  checkForUserInput();
  
  if (clearDataFlag){
    inCount = 0; 
    outCount = 0;
    clearDataFlag = false;    
  }

  int visible = 0;
  // Are we visible on sensor 1?
  visible = process_LIDAR(tfmP_1, smoothed_LIDAR_1, 100); 
  
  if (visible>=0){
    
    state = state + visible;

    // Are we visible on sensor 2?
    visible = process_LIDAR(tfmP_2, smoothed_LIDAR_2, 110);
    state = state + visible * 2;

    if (showRawData) dualPrintln(distanceThreshold);

    jsonPayload = "{\"deviceName\":\"" + deviceName +
                  "\",\"deviceMAC\":\"" + WiFi.macAddress();
  
    if ((previousState == 2) && (state == 3)){
      inCount += 1;
      jsonPayload = jsonPayload + "\",\"eventType\":\"inbound" +
                     "\",\"count\":\"" + inCount + "\"" +
                     "}";

      if (showRawData == false) dualPrintln(jsonPayload); 
    }
  
    if ((previousState == 1) && (state == 3)){
      outCount += 1;
      jsonPayload = jsonPayload + "\",\"eventType\":\"outbound" +
                    "\",\"count\":\"" + outCount + "\"" +
                    "}";

      if (showRawData == false) dualPrintln(jsonPayload);
    }
    
    previousState = state;
  }
}
