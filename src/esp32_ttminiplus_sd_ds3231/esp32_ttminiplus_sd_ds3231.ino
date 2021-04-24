/*
 * ParkData LIDAR Vehicle Counter
 * Copyright 2021, Digame Systems. All rights reserved.
 */


#define useRTC true
#define useWiFi true
#define useSDCard true


#include <TFMPlus.h>    // Include TFMini Plus LIDAR Library v1.4.0
#include <WiFi.h>       // WiFi stack
#include <HTTPClient.h> // To post to the ParkData Server
#include <DS3231.h>     // Real Time Clock Library
#include <Wire.h>       // 
#include "time.h"       // UTC functions
#include <SPI.h>        // SPI bus functions to talk to the SD Card
#include <SD.h>         // SD file handling
#include <CircularBuffer.h> // Adafruit library. Pretty small!

CircularBuffer<int, 100> buffer; // We're going to hang onto the last 100 points to visualize what the sensor sees

TFMPlus tfmP;           // Create a TFMini Plus object

RTClib myRTC;
float rtcTemp;

bool Century=false;
byte Year;
byte Month;
byte Date;
byte DoW;
byte Hour;
byte Minute;
byte Second; 

bool h12; //Hour counter
bool PM;

const int chipSelect = 4; // For the SPI and SD Card

// Parameters found in CONFIG.TXT on the SD card. 
String stringDeviceName;       
String stringDistThreshold; // Lane width parameter for counting
String stringOpMode;        // Currently only opmodeNetwork is supported
String stringSSID;          // Wireless network name. 
String stringPassword;      // Network PW
String stringServerURL;     // The ParkData server URL

float distanceThreshold = 400.0; // Maximum distance at which an object counts as 'present'

// Hardcoded time parameters. -- TODO: get some fallover time servers...
const char* ntpServer = "pool.ntp.org"; // Time server.
const long  gmtOffset_sec = 0;          // No offsets -- We're using GMT
const int   daylightOffset_sec = 0;

long msLastConnectionAttempt;  // Timer value of the last time we tried to connect to the wifi.

int LED_BUILTIN = 12; // Our built in indicator LED

// Aliases for easier reading
#define debugUART Serial
#define tfMiniUART Serial2   


//****************************************************************************************
// Silly Utility Function to format time values
// Format a byte as a left zero-padded, two-digit decimal string
String two_digits(byte value){
  String message = String(value, DEC);
  if (value <10) {
      message = "0" + message;
  }
  return message;
}

//****************************************************************************************
// Return the device's MAC address
String getMACAddress(){
  byte mac[6];

  WiFi.macAddress(mac);
  String retString= String(String(mac[5],HEX)+":");
  
  retString = String(retString + String(mac[4],HEX) +":");
  retString = String(retString + String(mac[3],HEX) +":");  
  retString = String(retString + String(mac[2],HEX) +":");
  retString = String(retString + String(mac[1],HEX) +":");
  retString = String(retString + String(mac[0],HEX));

  return retString;
}

//****************************************************************************************
// For us, Local Time will be UTC set by the NTP connection in setup.
String getLocalTime()
{
  String retStr;
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    retStr = String("Failed to obtain time");
    return retStr;
  }
  // Jared would like: 
  //YYYY-MM-DD HH:MM:SS
  //debugUART.print(&timeinfo, "%Y-%m-%d %H:%M:%S");
  retStr = String(timeinfo.tm_year+1900); //, "%Y-%m-%d %H:%M:%S");
  retStr = retStr + ("-");
  retStr = retStr + two_digits(timeinfo.tm_mon+1);
  retStr = retStr + ("-");
  retStr = retStr + two_digits(timeinfo.tm_mday);
  retStr = retStr + (" ");
  retStr = retStr + two_digits(timeinfo.tm_hour);
  retStr = retStr + (":");
  retStr = retStr + two_digits(timeinfo.tm_min);
  retStr = retStr + (":");
  retStr = retStr + two_digits(timeinfo.tm_sec);
  return retStr;
}



// Retrieve the time from the RTC and format 
String getRTCTime(){
    String message = "";
    DateTime now = myRTC.now();
    message += now.year();
    message += "-";
    message += two_digits(now.month());
    message += "-";
    message += two_digits(now.day());
    message +=" ";
    message += two_digits(now.hour());
    message += ':';
    message += two_digits(now.minute());
    message += ':';
    message += two_digits(now.second());
    return message;   
}

//****************************************************************************************
// Attempt to synchronize the RTC with the value set from the NTP synch at boot. 
// If the NTP was not set. Don't do anything and leave the current value in the RTC.
int setRTCTime(){
  
  String retStr;
  struct tm timeinfo;
  
  if(!getLocalTime(&timeinfo)){ // Read from the ESP32 RTC (not battery backed...) 
    retStr = String("Failed to obtain NTP time");
    return 1;
  }

  DS3231 clock;
  clock.setYear(timeinfo.tm_year-100);
  clock.setMonth(timeinfo.tm_mon+1);
  clock.setDate(timeinfo.tm_mday);
  clock.setHour(timeinfo.tm_hour);
  clock.setMinute(timeinfo.tm_min);
  clock.setSecond(timeinfo.tm_sec);
  
  return 0;
}  



//****************************************************************************************
// Attempt to connect to the WiFi Network.
bool connectToWiFi(){
  
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_OFF); // Let's start fresh...
  delay(1000);
  WiFi.begin(stringSSID.c_str(), stringPassword.c_str());
  long t1 = millis();
  bool timeOut = false;

  while ((WiFi.status() != WL_CONNECTED) && (millis() - t1 < 2000) ) {
    delay(500);
    debugUART.print(".");
  }
  if (millis()-t1>=2000){
    debugUART.print(" TIMEOUT!");
    return false;  
  } else{
    debugUART.println(" CONNECTED!");
    //Init and get the time from the NTP server
    String stringLocalTime="Failed to obtain NTP time";
    
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    stringLocalTime = getLocalTime();
    debugUART.print("NTP Time: ");
    debugUART.println(stringLocalTime);
      if (stringLocalTime != "Failed to obtain NTP time"){  
      debugUART.println("Synchronizing RTC w/ NTP Server...");
      setRTCTime(); // Set the RTC to the NTP value we just got.
    }
    debugUART.print("RTC Time: ");
    debugUART.println(getRTCTime()); 
    
    return true;
  }

  msLastConnectionAttempt=millis();
  
}

//****************************************************************************************
// Some of the most sophisticated code ever written.
void blinkLED(){
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                        // wait for a bit
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(100);  
}

//****************************************************************************************
// Grab network parameters from the SD Card.
void readDefaults(){
  debugUART.println("Reading default values...");
  debugUART.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin()) {
    debugUART.println("Card failed, or not present");
    // don't do anything more:
    // TODO: Fix this. 'Can't just stop in the real world.
    while (1);
  }
  debugUART.println(" Card initialized.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("/CONFIG.TXT");

  // if the file is available, read from to it:
  if (dataFile) {
      stringDeviceName = dataFile.readStringUntil('\r');
      stringDeviceName.trim();
      debugUART.print("Device Name: ");
      debugUART.println(stringDeviceName);

      stringDistThreshold = dataFile.readStringUntil('\r');
      stringDistThreshold.trim();
      debugUART.print("Distance Threshold: ");
      debugUART.println(stringDistThreshold);
      distanceThreshold = stringDistThreshold.toFloat();

      stringOpMode = dataFile.readStringUntil('\r');
      stringOpMode.trim();
      debugUART.print("Operating Mode: ");
      debugUART.println(stringOpMode);
      
      stringSSID = dataFile.readStringUntil('\r');
      stringSSID.trim();
      debugUART.print("SSID: ");
      debugUART.println(stringSSID);
      
      stringPassword = dataFile.readStringUntil('\r');
      stringPassword.trim();
      debugUART.print("Password: ");
      debugUART.println(stringPassword);
      
      stringServerURL = dataFile.readStringUntil('\r');
      stringServerURL.trim();
      debugUART.print("Server URL: ");
      debugUART.println(stringServerURL);
      
      dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    debugUART.println("Error opening NETWORK.TXT");
  }

}

//****************************************************************************************
// Device initialization                                   
//****************************************************************************************
void setup()
{
    // Ready the LED.
    pinMode(LED_BUILTIN, OUTPUT);
    
    debugUART.begin(115200);   // Intialize terminal serial port
    delay(1000);               // Give port time to initalize

    Wire.begin();

    debugUART.println("*****************************************************");
    debugUART.println("ParkData LIDAR Vehicle Counter");
    debugUART.println("Version 1.0");
    debugUART.println("Copyright 2021, Digame Systems. All rights reserved.");
    debugUART.println("*****************************************************");
    
    readDefaults(); // Grab the network parameters from the SD card

    //connect to WiFi
    debugUART.print("Connecting to: ");
    debugUART.print(stringSSID);
    while (!connectToWiFi()){};  // Spin until we have a connection. (A little risky... No network and we just sit here.)

    for(int i=0; i<5; i++){blinkLED();} // Indicate connection success.
    
    debugUART.print("MAC Address: ");
    debugUART.println(getMACAddress());

    tfMiniUART.begin(115200);  // Initialize TFMPLus device serial port.
    delay(1000);               // Give port time to initalize
    tfmP.begin(&tfMiniUART);   // Initialize device library object and...
                               // pass device serial port to the object.

    // Send some example commands to the TFMini-Plus
    // - - Perform a system reset - - 
    
    /*debugUART.printf( "Activating LIDAR Sensor... ");
    if( tfmP.sendCommand(SET_FRAME_RATE, FRAME_0)){
        debugUART.println("Set frame rate to 0hz.");
    }
    else tfmP.printReply();
    */
    
    debugUART.printf( "Activating LIDAR Sensor... ");
    if( tfmP.sendCommand(SYSTEM_RESET, 0)){
        debugUART.println("Sensor Active.");
    }
    else tfmP.printReply();

    debugUART.print("Delaying 5 seconds... ");
    
   // WiFi.mode(WIFI_OFF); // Let's start fresh...

    delay(5000);               // And wait a while.
    debugUART.println("Running!");
    
}

//****************************************************************************************
// Append a message to the DATALOG.TXT file on the SD card.
void appendDatalog(String jsonPayload){
  Serial.println("WiFi Disconnected. Saving to SD Card...");
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("/DATALOG.TXT", FILE_APPEND);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(jsonPayload);
    dataFile.close();
    // print to the serial port too:
    Serial.println(jsonPayload);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

//****************************************************************************************
// Save a JSON message to the server
void postJSON(String jsonPayload){
  HTTPClient http;
  
  // Your Domain name with URL path or IP address with path
  http.begin(stringServerURL.c_str());
  
  // If you need an HTTP request with a content type: application/json, use the following:
  http.addHeader("Content-Type", "application/json");
  debugUART.println(jsonPayload);
  blinkLED();
  
  long t1= millis();
  long t2 = t1;
  int httpResponseCode = http.POST(jsonPayload);
  
  debugUART.print("HTTP Response code: ");
  debugUART.println(httpResponseCode);
  t2=millis();
  //debugUART.print("Time to POST JSON: ");
  //debugUART.print(t2-t1);
  //debugUART.println(" msec");  
  // Free resources
  http.end();
  return;
}

//****************************************************************************************
// Save a JSON log file to the server
void postDatalog(){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("/DATALOG.TXT");

  String jsonPayload="";
  
  // if the file is available, read from to it:
  if (dataFile) {
     while (dataFile.available()) {
        jsonPayload = dataFile.readStringUntil('\r');
        jsonPayload.trim();
        postJSON(jsonPayload);
        //debugUART.println(jsonPayload);
     }
     debugUART.println("Done.");
     dataFile.close();

     SD.remove("/DATALOG.TXT"); // Delete the log after upload
     
     //debugUART.println(stringDeviceName);
  } else {
    debugUART.println("No DATALOG.TXT present.");  
  }
}



// Initialize some variables
int16_t tfDist = 0;    // Distance to object in centimeters
int16_t tfFlux = 0;    // Strength or quality of return signal
int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
float   smoothed = 0.0;

bool carPresent = false;    
bool lastCarPresent = false; 
int  carEvent = 0;

int lidarUpdateRate = 10;


//************************************************************************
//************************************************************************
void loop()
{
    delay(lidarUpdateRate);
    
    // Read the LIDAR Sensor
    if( tfmP.getData( tfDist, tfFlux, tfTemp)) { 

      //Filter the measured distance
      smoothed = smoothed *0.5 + (float)tfDist * 0.5;
      int intSmoothed = (int) smoothed*10;

      buffer.push(intSmoothed);
      

      if (smoothed < distanceThreshold) {
        carPresent = true; 
      } else {
        carPresent = false; 
      }

      //debugUART.print(state);
      //debugUART.print(" ");
      /*
      debugUART.print(tfDist);
      debugUART.print(" ");
      debugUART.print(smoothed);
      debugUART.print(" ");
      debugUART.print(carEvent);
      debugUART.println();
      */  

     if ((lastCarPresent == true) && (carPresent == false)){ // The car has left the field of view. 
         
        carEvent = 500;
        
        String jsonPayload = "{\"deviceName\":\"" + stringDeviceName + 
                               "\",\"deviceMAC\":\"" + getMACAddress() + 
                               "\",\"timeStamp\":\""+ getRTCTime() + 
                               "\",\"eventType\":\"vehicle" +
                               "\",\"rawSignal\":[";

        using index_t = decltype(buffer)::index_t;
        for (index_t i = 0; i < buffer.size(); i++) {
            jsonPayload = jsonPayload + buffer[i]; 
            if (i<buffer.size()-1){ 
              jsonPayload = jsonPayload + ",";
            }
        }

        jsonPayload = jsonPayload + "]}";
          
        if(WiFi.status()== WL_CONNECTED){
            // We have a WiFi connection. -- Upload the data to the the server. 
            postJSON(jsonPayload);
            
        } else {
            appendDatalog(jsonPayload);

            // Try connecting every five minutes 
            if ((millis() - msLastConnectionAttempt) > (5*60*1000)){ 
              //Try to connect up to two times...
              connectToWiFi();    // Once
              
              if (WiFi.status() != WL_CONNECTED){
                connectToWiFi();  // Twice
              }
              
              if (WiFi.status() == WL_CONNECTED){
                postDatalog();  // POSTS and clears out the log.
              }
           }
        }
       
     } else {
      
        carEvent = 0;
        
     }

    lastCarPresent = carPresent;
  }

}
