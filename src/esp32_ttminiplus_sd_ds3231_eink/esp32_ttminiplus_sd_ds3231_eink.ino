/*
 * ParkData Traffic Monitoring Platform
 * Version 0.9
 * 
 * A traffic counting system that uses LIDAR to track pedestrians and vehicles.
 * 
 * Copyright 2021, Digame Systems. All rights reserved.
 */

#define USE_EINK false
#define ADAFRUIT_EINK_SD true  // Some Adafruit Eink Displays have an integrated SD card
#define USE_WIFI true
#define SHOW_DATA_STREAM false // A debugging flag to show raw LIDAR values on the serial monitor
#define APPEND_RAW_DATA true   // Add a 100 pts of raw LIDAR data to the JSON message
 
#include <TFMPlus.h>           // Include TFMini Plus LIDAR Library v1.4.0

#if USE_WIFI
  #include <WiFi.h>            // WiFi stack
  #include <HTTPClient.h>      // To post to the ParkData Server
#endif

#include <Wire.h>              // 
#include <SPI.h>               // SPI bus functions to talk to the SD Card
#include <SD.h>                // SD file handling
#include <CircularBuffer.h>    // Adafruit library. Pretty small!

#include "digameTime.h"        // Digame Time Functions.
#include "digameNetwork.h"     // Digame Network Functions

#if USE_EINK
  #include "digameDisplay.h" // Digame eInk Display Functions.
#endif

// Aliases for easier reading
#define debugUART Serial
#define tfMiniUART Serial2  

#if ADAFRUIT_EINK_SD
  #define SD_CS       14  // SD card chip select
#else
  #define SD_CS       4
#endif

const int samples = 100;
CircularBuffer<int, samples> buffer; // We're going to hang onto the last 100 points to visualize what the sensor sees
float data[samples];

SemaphoreHandle_t mutex_v; // Mutex used to protect our msgBuffer (see below).
CircularBuffer<String, samples> msgBuffer; // A buffer containing JSON messages to be sent to the server...

TFMPlus tfmP;           // Create a TFMini Plus object

// HW Status Variables -- sniffed in setup()
bool sdCardPresent = false;
bool rtcPresent    = false;
bool lidarPresent  = false; 
bool wifiConnected = false;

// Parameters found in CONFIG.TXT on the SD card. Fails over to these values if not found.
String stringDeviceName    = "Digame Systems";       
String stringDistThreshold = "300";           // Lane width parameter for counting
String stringOpMode        = "opModeNetwork"; // Currently only opmodeNetwork is supported
String stringSSID          = "Bighead";       // Wireless network name. 
String stringPassword      = "billgates";     // Network PW
String stringServerURL     = "https://trailwaze.info/zion/lidar_sensor_import.php";     // The ParkData server URL
float distanceThreshold    = 300.0;           // Maximum distance at which an object counts as 'present'

long msLastConnectionAttempt;  // Timer value of the last time we tried to connect to the wifi.

int LED_DIAG = 12;  // Indicator LED
int CTR_RESET = 32; // Counter Reset Input

int heartbeatTime;    // Issue Heartbeat message once an hour. Holds the current Minute.
int oldHeartbeatTime; // Value the last time we looked.

// LIDAR signal analysis parameters
int16_t tfDist        = 0;    // Distance to object in centimeters
int16_t tfFlux        = 0;    // Strength or quality of return signal
int16_t tfTemp        = 0;    // Internal temperature of Lidar sensor chip
float smoothed        = 0.0;
bool  carPresent      = false;    
bool  lastCarPresent  = false; 
int   carEvent        = 0;
int   lidarUpdateRate = 10;
float correl1 = 0.0;

// Messaging flags
bool jsonPostNeeded = false;
bool bootMessageNeeded = true;
bool heartbeatMessageNeeded = false;
bool vehicleMessageNeeded = false;

// The message being sent
String jsonPayload;

// The minute (0-59) within the hour we woke up at boot. 
int bootMinute;


//****************************************************************************************
// Some of the most sophisticated code ever written.
void blinkLED(){
  digitalWrite(LED_DIAG, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                     // wait for a bit
  digitalWrite(LED_DIAG, LOW);    // turn the LED off by making the voltage LOW
  delay(100);  
}


//****************************************************************************************
// See if the card is present and can be initialized.
bool initSDCard(){
  if (!SD.begin(SD_CS)) {
    return false;
  }  else {
    return true;    
  }
}


//****************************************************************************************
// Debug dump of current values for program defaults.
void showDefaults(){
  
  debugUART.print("      Device Name: ");
  debugUART.println(stringDeviceName);
  
  debugUART.print("      Distance Threshold: ");
  debugUART.println(stringDistThreshold);
  distanceThreshold = stringDistThreshold.toFloat();
  
  debugUART.print("      Operating Mode: ");
  debugUART.println(stringOpMode);
  
  debugUART.print("      SSID: ");
  debugUART.println(stringSSID);
  
  debugUART.print("      Password: ");
  debugUART.println(stringPassword);
  
  debugUART.print("      Server URL: ");
  debugUART.println(stringServerURL);

}


//****************************************************************************************
// Grab program parameters from the SD Card.
bool readDefaults(){

  File dataFile = SD.open("/CONFIG.TXT");

  // If the file is available, read from it:
  if (dataFile) {
      stringDeviceName = dataFile.readStringUntil('\r');
      stringDeviceName.trim();
      
      stringDistThreshold = dataFile.readStringUntil('\r');
      stringDistThreshold.trim();
      distanceThreshold = stringDistThreshold.toFloat();

      stringOpMode = dataFile.readStringUntil('\r');
      stringOpMode.trim();
      
      stringSSID = dataFile.readStringUntil('\r');
      //stringSSID = "AndroidAP3AE2";   // ***********************************REMOVE LATER!!!
      stringSSID.trim();
            
      stringPassword = dataFile.readStringUntil('\r');
      //stringPassword = "ohpp8971";    //*************************************REMOVE LATER!!!
      stringPassword.trim();
      
      stringServerURL = dataFile.readStringUntil('\r');
      stringServerURL.trim();
      
      dataFile.close();
      return true;
  }
  // If the file isn't open, pop up an error:
  else {
    debugUART.println("      ERROR! Trouble opening CONFIG.TXT");
    return false;
  }
}


//****************************************************************************************
// Device initialization                                   
//****************************************************************************************
void setup()
{
    String compileDate = F(__DATE__);
    String compileTime = F(__TIME__);

    String stat="";
    
    // Ready the LED and RESET pins
    pinMode(LED_DIAG, OUTPUT);
    pinMode(CTR_RESET,INPUT_PULLUP);
       
    debugUART.begin(115200);   // Intialize terminal serial port
    delay(1000);               // Give port time to initalize

    Wire.begin();

#if USE_EINK
    display.init(0);
    // first update should be full refresh
    initDisplay();
    displaySplashScreen();
    delay(5000);
    displayInitializingScreen();
#endif 

    debugUART.println("*****************************************************");
    debugUART.println("ParkData Traffic Monitoring Platform");
    debugUART.println("Version 0.9");
    debugUART.println("Copyright 2021, Digame Systems. All rights reserved.");
    debugUART.println();
    debugUART.print("Compiled on ");
    debugUART.print(compileDate);
    debugUART.print(" at ");
    debugUART.println(compileTime); 
    debugUART.println("*****************************************************");
    debugUART.println();
    debugUART.println("HARDWARE INITIALIZATION");
    debugUART.println();

    debugUART.print("  Testing for SD Card Module... ");
    sdCardPresent = initSDCard();
    if ((sdCardPresent) && (readDefaults())){
      debugUART.println("Module found. (Parameters read from SD Card.)");
      stat += "  SD   :  OK\n";
    }else{
      debugUART.println("ERROR! Module NOT found. (Parameters set to default values.)");    
      stat += "  SD   :  ERROR!\n";
    }

    showDefaults();

#if USE_WIFI
    debugUART.print("  Testing for WiFi connectivity... ");
    wifiConnected = initWiFi(stringSSID, stringPassword);
    msLastConnectionAttempt=millis();  // Save the time of the latest connection attempt. Used to schedule retries if lost.
    debugUART.println();
    if (wifiConnected){
      debugUART.println("      Connection established.");
      stat +="  WiFi :  OK\n";
      for(int i=0; i<5; i++){blinkLED();} // Indicate connection success. 
    }else{
      debugUART.println("      ERROR! Could NOT establish WiFi connection. (Credentials OK?)");  
      stat +="  WiFi :  ERROR!\n";  
      for(int i=0; i<5; i++){blinkLED(); delay(1000);} // Indicate connection failure. 
    }

    debugUART.print("  Reading Device MAC Address... ");
    debugUART.println(getMACAddress());
#endif

    tfMiniUART.begin(115200);  // Initialize TFMPLus device serial port.
    delay(1000);               // Give port time to initalize
    tfmP.begin(&tfMiniUART);   // Initialize device library object and...
                               // pass device serial port to the object.

    // Send some commands to the TFMini-Plus
    // - - Perform a system reset - - 
    
    /*debugUART.printf( "Activating LIDAR Sensor... ");
    if( tfmP.sendCommand(SET_FRAME_RATE, FRAME_0)){
        debugUART.println("Set frame rate to 0hz.");
    }
    else tfmP.printReply();
    */
    
    debugUART.printf( "  Testing for LIDAR Sensor... ");
    if( tfmP.sendCommand(SYSTEM_RESET, 0)){
        debugUART.println("LIDAR Sensor initialized.");
        stat +="  LIDAR:  OK\n";
    }
    else {
        debugUART.println("ERROR! LIDAR Sensor not found or sensor error.");
        stat +="  LIDAR:  ERROR!\n";
        tfmP.printReply();
    }

    debugUART.print("  Testing for Real-Time-Clock module... ");
    rtcPresent = initRTC();
    if (rtcPresent){
      debugUART.println("RTC found. (Program will use time from RTC.)");
      stat +="  RTC  :  OK\n";
      debugUART.print("      RTC Time: ");
      debugUART.println(getRTCTime()); 
    }else{
      debugUART.println("ERROR! Could NOT find RTC. (Program will attempt to use NTP time.)");   
      stat +="   RTC  :  ERROR!\n"; 
    }

    if (wifiConnected){ // Attempt to synch ESP32 clock with NTP Server...
      synchTime(rtcPresent);
    }

    bootMinute = getRTCMinute();
    heartbeatTime = bootMinute;
    oldHeartbeatTime = heartbeatTime;

    stat += "  LoRa :  OK\n";
#if USE_EINK
    displayStatusScreen(stat);
#endif
    //debugUART.println(stat);
    delay(3000);
    debugUART.println();
    debugUART.println("RUNNING!");  
    debugUART.println(); 

    mutex_v = xSemaphoreCreateMutex();  //The mutex we will use to protect the msgBuffer
    xTaskCreate(
      messageManager,    // Function that should be called
      "Message Manager",   // Name of the task (for debugging)
      5000,            // Stack size (bytes)
      NULL,            // Parameter to pass
      1,               // Task priority
      NULL             // Task handle
    );

#if USE_EINK
    displayCountScreen(0);
    showValue(0);
#endif

}


//****************************************************************************************
// Append a message to the DATALOG.TXT file on the SD card.
void appendDatalog(String jsonPayload){
  if (sdCardPresent){
    File dataFile = SD.open("/DATALOG.TXT", FILE_APPEND);
  
    // If the file is available, write to it:
    if (dataFile) {
      dataFile.println(jsonPayload);
      dataFile.close();
      // print to the serial port too:
      Serial.println("Appending data file on SD:");
      Serial.println(jsonPayload);
    }
    // If the file isn't open, pop up an error:
    else {
      Serial.println("ERROR! Trouble opening datalog.txt");
    }
  } else {
    Serial.println("ERROR! SD Card not present.");  
  }
}

//****************************************************************************************
// Save a single JSON message to the server.
void postJSON(String jsonPayload){

#if USE_WIFI
  HTTPClient http;
  
  // Your Domain name with URL path or IP address with path
  http.begin(stringServerURL.c_str());
  
  // If you need an HTTP request with a content type: application/json, use the following:
  http.addHeader("Content-Type", "application/json");

  int httpResponseCode = http.POST(jsonPayload);
  
  #if SHOW_DATA_STREAM
  #else
    debugUART.print("HTTP response code: ");
    debugUART.println(httpResponseCode);
  #endif

  // Free resources
  http.end();
  
#endif // USE_WIFI


  return;
}


//****************************************************************************************
// Save a JSON log file to the server
void postDatalog(){
  if (sdCardPresent){
    String jsonPayload="";
    
    File dataFile = SD.open("/DATALOG.TXT");
  
    // If the file is available, read from it:
    if (dataFile) {
       while (dataFile.available()) {
          jsonPayload = dataFile.readStringUntil('\r');
          jsonPayload.trim();
          postJSON(jsonPayload);
       }
       debugUART.println("Done.");
       dataFile.close();
       SD.remove("/DATALOG.TXT"); // Delete the log after upload   
    } else {
      // No file.
      debugUART.println("No DATALOG.TXT present.");  
    }
  } else {
    debugUART.println("ERROR! No SD card present.");
  }
}


//****************************************************************************************
// JSON messages to the server all have a similar format. 
String buildJSONHeader(String eventType){
  String jsonHeader;
  if (rtcPresent){
    jsonHeader = "{\"deviceName\":\"" + stringDeviceName + 
                   "\",\"deviceMAC\":\"" + getMACAddress() + 
                   "\",\"timeStamp\":\"" + getRTCTime() + 
                   "\",\"eventType\":\"" + eventType + 
                   "\"";
  } else {
    jsonHeader = "{\"deviceName\":\"" + stringDeviceName + 
                   "\",\"deviceMAC\":\"" + getMACAddress() + 
                   "\",\"timeStamp\":\"" + getLocalTime() + 
                   "\",\"eventType\":\"" + eventType + 
                   "\"";  
  }                    
  return jsonHeader;
}


//************************************************************************
//We have a JSON message to send to the server. Try to send it. If it fails,
//Save it to the SD card for later delivery when we can reconnect.
void processMessage(String jsonPayload){
  
#if USE_WIFI
  if(WiFi.status()== WL_CONNECTED){
      // We have a WiFi connection. -- Upload the data to the the server. 
      postJSON(jsonPayload);
      
  } else {
      // No WiFi -- Save locally.
      appendDatalog(jsonPayload);
  
      // Try connecting every five minutes 
      if ((millis() - msLastConnectionAttempt) > (5*60*1000)){ 
        //Try to connect up to two times...
        connectToWiFi(stringSSID, stringPassword);    // Once
        
        if (WiFi.status() != WL_CONNECTED){
          connectToWiFi(stringSSID, stringPassword);  // Twice
        }

        msLastConnectionAttempt=millis();
        
        if (WiFi.status() == WL_CONNECTED){
          postDatalog();  // POSTS and clears out the log.
        }
     }
  }
#endif

}


//****************************************************************************************
// Math functions
//****************************************************************************************

float mean(float a[], int numSamples){
    float result;
    
    result = 0;   
    for (int i=0; i<numSamples; i++) {
      result += a[i];
    }
    result /= numSamples;
    return result;  
}

// Calculate the correlation coefficient between two arrays
float correlation(float x[], float y[], int numSamples){ 
    float sx, sy, sxy, denom; 
    float mx, my; // means
    float r; // correlation coefficient

    /* Calculate the means */
    mx = mean(x, numSamples);
    my = mean(y, numSamples);
    
    /* Calculate the denominator */
    sx = 0;
    sy = 0;
    
    for (int i=0; i<numSamples; i++) {
      sx += (x[i] - mx) * (x[i] - mx);
      sy += (y[i] - my) * (y[i] - my);
    }
    
    denom = sqrt(sx*sy);

   /* Calculate the correlation coefficient */
    sxy = 0;
    for (int i=0; i<numSamples; i++) {
          sxy += (x[i] - mx) * (y[i] - my); 
    }
    r = sxy / denom;
    return r;
}

//************************************************************************
// An attempt to improve on the very simple thresholding scheme in processLIDARSignal.
// This routine uses correlation to look for the presence of a rising/falling edge
// in the data. Rising edges are associated with 'vehicle left' events. 
bool processLIDARSignalCorrel(){ // Returns true if vehicle signature detected, false otherwise.

  bool retValue = false;
  
  //Falling Edge Model 100 pts
  float fallingEdgeModel[] = {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

  // correlation results in -100 to 100 (perfect anti to perfect correlation)
  // we don't need perfect for detection.
  int16_t highThreshold = 30;
  int16_t lowThreshold = -30; 
  
  int16_t highEventValue = highThreshold;
  int16_t lowEventValue  = lowThreshold;
  
  delay(lidarUpdateRate);
  
  // Read the LIDAR Sensor
  if( tfmP.getData(tfDist, tfFlux, tfTemp) ) { 
    
    buffer.push(tfDist);
    
    for (byte i = 0; i < buffer.size(); i++) {
      data[i]=buffer[i];
    }
    
    data[0]=1; // Needed to keep denom in correl above from getting too small and introducing big variations.

    correl1 = (correlation(fallingEdgeModel, data, samples)* 100)-10;// not sure why we need this to be centered on zero
    
    //Filter the correlated value
    smoothed = smoothed * 0.8 + (float)correl1 * 0.2;

    if (smoothed < lowThreshold){
      lowEventValue = lowThreshold + 100;   
      digitalWrite(32, HIGH);
      if (buffer.size()==samples){
        retValue = true;
      }
    } else {
      lowEventValue = lowThreshold;  
      digitalWrite(32, LOW);
    }

    if (smoothed>  highThreshold){
      highEventValue = highThreshold + 100;   
    } else {
      highEventValue = highThreshold;  
    }
 
    /*// For charting / debugging     
    debugUART.print(tfDist+300);
    debugUART.print(" ");
    debugUART.print(smoothed);
    debugUART.print(" ");
    debugUART.print(lowEventValue);
    debugUART.print(" ");
    debugUART.print(highEventValue);
    debugUART.print(" ");
    debugUART.print(100);
    debugUART.print(" ");
    debugUART.print(-100);
    debugUART.println();
    */
    
  }
  return retValue; 
}

//************************************************************************
// A dirt-simple processing scheme based on a hard threshold read from the
// SD card. Pretty susceptible to noisy conditions. TODO: Improve. 
bool processLIDARSignal(){
  
    bool retValue = false;
    
    delay(lidarUpdateRate);
    
    // Read the LIDAR Sensor
    if( tfmP.getData( tfDist, tfFlux, tfTemp)) { 

      //Filter the measured distance
      smoothed = smoothed *0.75 + (float)tfDist * 0.25;
      int intSmoothed = (int) smoothed*10;

      buffer.push(intSmoothed);
      
      if (smoothed < distanceThreshold) {
        carPresent = true; 
      } else {
        carPresent = false; 
      }

     if ((lastCarPresent == true) && (carPresent == false)){ // The car has left the field of view.   
        carEvent = 500;
        retValue = true;
          
     } else {    
        carEvent = 0;   
     }

#if SHOW_DATA_STREAM
     debugUART.print(tfDist);
     debugUART.print(",");
     debugUART.print(smoothed);
     debugUART.print(",");
     debugUART.print(distanceThreshold);
     debugUART.print(",");
     debugUART.println(carEvent);
#endif
     
     lastCarPresent = carPresent;
  }

    
  return retValue;  
}


/********************************************************************************/
// Experimenting with using a circular buffer to enqueue messages to the server...
void messageManager(void *parameter){
  String activeMessage;
  
  for(;;){  
    if (msgBuffer.size()>0){

#if SHOW_DATA_STREAM
#else
      Serial.print("MessageManager: msgBuffer.size(): ");
      Serial.println(msgBuffer.size());
      Serial.println("MessageManager: Sending message: ");
#endif      
      xSemaphoreTake(mutex_v, portMAX_DELAY); 
        //Serial.println("  MessageManager: Shifting msgBuffer..."); 
        activeMessage=msgBuffer.shift();
      xSemaphoreGive(mutex_v);
#if SHOW_DATA_STREAM
#else
      Serial.println(activeMessage);
#endif
      processMessage(activeMessage);        
    } else{
      //Serial.println("MessageManager: Nothing to do...");      
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);   
  }   
}

//************************************************************************
// Main Loop
//************************************************************************

double count = 0;

void loop()
{ 
  //DATA ACQUISITION
  
    // Evaluate the LIDAR signal to determine if a vehicle passing event has occured.
    // Different evaluation modes are under consideration...
    if (stringOpMode=="opmodeCorrelation"){      
        vehicleMessageNeeded = processLIDARSignalCorrel(); //Correlation Detection
    } else{ 
      if (stringOpMode=="opmodeThreshold"){
        vehicleMessageNeeded = processLIDARSignal(); // Threshold Detection
      } else { 
        vehicleMessageNeeded = processLIDARSignal(); // UKNOWN. Default to Threshold Detection
      }
    } 

   // Check for RESET button being pressed
   if (digitalRead(CTR_RESET)== LOW) {
     count = 0;
     #if SHOW_DATA_STREAM
     #else
        debugUART.print("Loop: RESET button pressed. Count: ");
        debugUART.println(count);  
     #endif
     #if USE_EINK
       initDisplay();
       displayCountScreen(count);
       showValue(count);
     #endif
   }

  //MESSAGE HANDLING
  
    // Runs once at startup.
    if (bootMessageNeeded){
      jsonPayload = buildJSONHeader("boot");
      jsonPayload = jsonPayload + "}";
      jsonPostNeeded = true;
      bootMessageNeeded = false;
    }

    // Issue a heartbeat message every hour.
    heartbeatTime = getRTCMinute();
    if ((oldHeartbeatTime != bootMinute) && (heartbeatTime == bootMinute)){heartbeatMessageNeeded = true;}  
    if (heartbeatMessageNeeded){
      jsonPayload = buildJSONHeader("heartbeat");
      jsonPayload = jsonPayload + "}";
      jsonPostNeeded = true;
      heartbeatMessageNeeded = false;
    }
    oldHeartbeatTime = heartbeatTime;
    
    if (vehicleMessageNeeded){
      
      if (buffer.size()==samples){
        
        count++;
        
        #if SHOW_DATA_STREAM
        #else
          debugUART.print("Loop: Count event. Counts: ");
          debugUART.println(count);
        #endif 
             
        #if USE_EINK
          //displayCount(count);
          showValue(count);
        #endif

        jsonPayload = buildJSONHeader("vehicle");
        jsonPayload = jsonPayload + ",\"operatingMode\":\"" + stringOpMode +"\"";
        
        #if APPEND_RAW_DATA
        // Vehicle passing event messages may include raw data from the sensor.
        // If so, tack the data buffer on the JSON message.
        jsonPayload = jsonPayload + ",\"rawSignal\":[";
        using index_t = decltype(buffer)::index_t;
        for (index_t i = 0; i < buffer.size(); i++) {
          jsonPayload = jsonPayload + buffer[i]; 
          if (i<buffer.size()-1){ 
            jsonPayload = jsonPayload + ",";
          }
        }
        #endif
        
        jsonPayload = jsonPayload + "]}";
        jsonPostNeeded = true; 
      }
      vehicleMessageNeeded = false; 
    }
     
    // Push the JSON Payload into the message buffer. The messageManager task will handle it. 
    if (jsonPostNeeded){
      
      xSemaphoreTake(mutex_v, portMAX_DELAY); 
        #if SHOW_DATA_STREAM
        #else
          Serial.println("Loop: Pushing msgBuffer...");
        #endif         
        msgBuffer.push(jsonPayload); 
      xSemaphoreGive(mutex_v);
      
      //processMessage(jsonPayload); 
      jsonPostNeeded = false; 
      
      if (stringOpMode=="opmodeCorrelation"){ 
        buffer.clear();  // TODO: think about a way to do the POST in a separate thread so we can continuously gather data...
      }
      
    }   
}
