/* SET RTC: A simple utility to set time in a battery-backed RTC (DS3231)
 *  from an NTP time server.
 *  
 *  Designed for ESP32-based systems
 *  
 *  Copyright 2021, Digame systems. All rights reserved. 
 */
 
#define debugUART Serial

#include <digameJSONConfig.h>   // Config Struct
#include <digameTime.h>         // Digame Time Functions
#include <digameNetwork.h>      // Digame Network Functions

// Globals
String swVersion = "1.0.0";

Config config;

// Declares
void  initPorts(); // Set up the Serial port
void  splash();    // Show a boot message on the Serial port 
//void  enableWiFi(char*,char*); // Log into the network using our SSID/PW


//************************************************************************
void initPorts(){
  
  debugUART.begin(115200); 
  delay(1000);
  Wire.begin();
  
}


//************************************************************************
void splash(){
  
  String compileDate = F(__DATE__);
  String compileTime = F(__TIME__);

  debugUART.println("*****************************************************");
  debugUART.println("SET RTC: A utility to set the time on a DS3231 RTC");
  debugUART.println(" module using NTP.");
  debugUART.println("Version " + swVersion);
  debugUART.println("Copyright 2021, Digame Systems. All rights reserved.");
  debugUART.println();
  debugUART.print("Compiled on ");
  debugUART.print(compileDate);
  debugUART.print(" at ");
  debugUART.println(compileTime); 
  debugUART.println("*****************************************************");
  debugUART.println();

}


//************************************************************************
// Setup
//************************************************************************
void setup() {
  
  initPorts(); // Set up serial ports
  
  splash();   
  
  debugUART.println("INITIALIZING\n");

  config.ssid       = (const char *)"Bighead";   // YOUR_NETWORK_NAME 
  config.password   = (const char *)"billgates"; // YOUR_PASSWORD

  enableWiFi(config);
  
  debugUART.println("  MAC Address: " + getMACAddress());  
  
  debugUART.println("Testing for Real-Time-Clock module...");
  if (rtcPresent()){
    debugUART.print("  RTC found. Time: ");
    debugUART.println(getRTCTime()); 
  }else{
    debugUART.println("  ERROR! Could NOT find RTC.");   
  }

  if (wifiConnected){ // Attempt to synch ESP32 and DS3231 with NTP server
    synchTimesToNTP();
  }
    
  debugUART.println();
  debugUART.println("RUNNING\n");
  
}


//************************************************************************
// Main Loop
//************************************************************************
void loop() {
  
  debugUART.println(getRTCTime());
  
  delay(1000);
  
}
