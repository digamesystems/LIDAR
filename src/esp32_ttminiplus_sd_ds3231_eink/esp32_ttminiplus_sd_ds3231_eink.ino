/*
 * HEIMDALL VCS - Vehicle Counting Systems 
 * 
 * A traffic counting system that uses LIDAR to track pedestrians and vehicles.
 * 
 * Copyright 2021, Digame Systems. All rights reserved.
 */

// Mix-n-match hardware configuration
#define USE_WIFI false          // WiFi back haul
#define USE_LORA true           // Reyax896 LoRa Module back haul
#define USE_BLUETOOTH false     // Bluetooth back haul (Not implemented, yet.)
#define USE_EINK true           // Adafruit eInk Display
#define ADAFRUIT_EINK_SD true   // Some Adafruit Eink Displays have an integrated SD card so we don't need a separate module
#define SHOW_DATA_STREAM false  // A debugging flag to show raw LIDAR values on the serial monitor
#define APPEND_RAW_DATA false   // Add a 100 pts of raw LIDAR data to the JSON message
 
#include <TFMPlus.h>            // Include TFMini Plus LIDAR Library v1.4.0

#include <WiFi.h>               // WiFi stack

#if USE_WIFI
  #include <HTTPClient.h>       // To post to the ParkData Server
#endif

#include <CircularBuffer.h>     // Adafruit library. Pretty small!

#include "digameTime.h"         // Digame Time Functions
#include "digameMath.h"         // Mean and correlation
#include "digameConfig.h"       // Program parameters from config file on SD card
#include "digameNetwork.h"    // Digame Network Functions

#if USE_EINK
  #include "digameDisplay.h"    // Digame eInk Display Functions.
#endif

#include "driver/adc.h"
#include <esp_bt.h>

// Aliases for easier reading
#define debugUART  Serial
#define tfMiniUART Serial2  

#if USE_LORA
  #define LoRaUART   Serial1
#endif


const int samples = 100;
CircularBuffer<int, samples> buffer; // We're going to hang onto the last 100 points to visualize what the sensor sees
float data[samples];

SemaphoreHandle_t mutex_v; // Mutex used to protect our jsonMsgBuffers (see below).

CircularBuffer<String, samples> jsonMsgBuffer; // A buffer containing JSON messages to be sent to the server.

#if USE_LORA
  CircularBuffer<String, samples> loraMsgBuffer; // A buffer containing JSON messages to be sent to the LoRa basestation.
#endif

TFMPlus tfmP;           // Create a TFMini Plus object

// HW Status Variables -- sniffed in setup()
bool sdCardPresent = false;
bool rtcPresent    = false;
bool lidarPresent  = false; 
bool wifiConnected = false;

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
String jsonPayload; //A fat JSON payload for the server. 
String loraPayload; //A stripped down version for LoRa

// The minute (0-59) within the hour we woke up at boot. 
int bootMinute;


void enableWiFi(){
    adc_power_on();
    delay(200);
 
    WiFi.disconnect(false);  // Reconnect the network
    WiFi.mode(WIFI_STA);    // Switch WiFi off
 
    delay(200);
 
    //debugUART.println("START WIFI");
    WiFi.begin(params.networkName.c_str(), params.password.c_str());
 
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        //debugUART.print(".");
    }
 
    //debugUART.println("");
    //debugUART.println("WiFi connected");
    //debugUART.println("IP address: ");
    //debugUART.println(WiFi.localIP());
    
}

void disableWiFi(){
    adc_power_off();
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF);    // Switch WiFi off
    //debugUART.println("");
    //debugUART.println("WiFi disconnected!");
}

void disableBluetooth(){
    // Rather disappointing on power improvement.
    btStop();
    //debugUART.println("");
    //debugUART.println("Bluetooth stop!");
}
 
void setLowPowerMode() {
    debugUART.println("Setting Power Mode:");
    disableWiFi();
    disableBluetooth();
    setCpuFrequencyMhz(40);
    debugUART.println("  Low Power Mode Enabled.");
}

 
void setNormalPowerMode() {
    setCpuFrequencyMhz(240);
    enableWiFi(); 

    debugUART.println("Setting Power Mode:");
    // Wake up LoRa module.
    LoRaUART.println("AT");   
    debugUART.println("  Normal Power Mode Enabled.");
}


//****************************************************************************************
// Some of the most sophisticated code ever written.
void blinkLED(){
  digitalWrite(LED_DIAG, HIGH);   
  delay(100);                     
  digitalWrite(LED_DIAG, LOW);    
  delay(100);  
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

#if USE_LORA
    LoRaUART.begin(115200, SERIAL_8N1, 25, 33);
#endif
    
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
    debugUART.println("Version 0.9.2");
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

    setLowPowerMode();

    debugUART.print("  Testing for SD Card Module... ");
    sdCardPresent = initSDCard();
    if ((sdCardPresent) && (readDefaults())){
      debugUART.println("Module found. (Parameters read from SD Card.)");
      stat += "   SD   : OK\n";
    }else{
      debugUART.println("ERROR! Module NOT found. (Parameters set to default values.)");    
      stat += "   SD   : ERROR!\n";
    }

    showDefaults();

#if USE_WIFI
    debugUART.print("  Testing for WiFi connectivity... ");
    wifiConnected = initWiFi(params.networkName, params.password);
    msLastConnectionAttempt=millis();  // Save the time of the latest connection attempt. Used to schedule retries if lost.
    debugUART.println();
    if (wifiConnected){
      debugUART.println("      Connection established.");
      stat +="   WiFi : OK\n";
      for(int i=0; i<5; i++){blinkLED();} // Indicate connection success. 
    }else{
      debugUART.println("      ERROR! Could NOT establish WiFi connection. (Credentials OK?)");  
      stat +="   WiFi : ERROR!\n";  
      for(int i=0; i<5; i++){blinkLED(); delay(1000);} // Indicate connection failure. 
    }

    debugUART.print("  Reading Device MAC Address... ");
    debugUART.println(getMACAddress());
#else
     //debugUART.println("Turning off WiFi...");
     WiFi.mode(WIFI_OFF); 
#endif

    btStop();

    tfMiniUART.begin(115200);  // Initialize TFMPLus device serial port.
    delay(1000);               // Give port time to initalize
    tfmP.begin(&tfMiniUART);   // Initialize device library object and...
                               // pass device serial port to the object.

    // Send some commands to the TFMini-Plus
    // - - Perform a system reset - - 
    
    debugUART.printf( "  Activating LIDAR Sensor... ");
    
    //debugUART.printf( "  Testing for LIDAR Sensor... ");
    if( tfmP.sendCommand(SYSTEM_RESET, 0)){
        debugUART.println("LIDAR Sensor initialized.");
        stat +="   LIDAR: OK\n";

        delay(500);
        
        //debugUART.printf( "  Adjusting Frame Rate... ");
        //if( tfmP.sendCommand(SET_FRAME_RATE, FRAME_0)){ //FRAME_0 is triggered mode.
        //   debugUART.println("  Frame Rate Adjusted.");
        // }
        // else tfmP.printReply(); 
    }
    else {
        debugUART.println("ERROR! LIDAR Sensor not found or sensor error.");
        stat +="   LIDAR: ERROR!\n";
        tfmP.printReply();
    }

    debugUART.print("  Testing for Real-Time-Clock module... ");
    rtcPresent = initRTC();
    if (rtcPresent){
      debugUART.println("RTC found. (Program will use time from RTC.)");
      stat +="   RTC  : OK\n";
      debugUART.print("      RTC Time: ");
      debugUART.println(getRTCTime()); 
    }else{
      debugUART.println("ERROR! Could NOT find RTC. (Program will attempt to use NTP time.)");   
      stat +="    RTC  : ERROR!\n"; 
    }

    if (wifiConnected){ // Attempt to synch ESP32 clock with NTP Server...
      synchTime(rtcPresent);
    }

    bootMinute = getRTCMinute();
    heartbeatTime = bootMinute;
    oldHeartbeatTime = heartbeatTime;

    #if USE_LORA
      stat += "   LoRa : OK\n";
    #endif
    
    #if USE_EINK
      displayStatusScreen(stat);
    #endif

    delay(3000);
    debugUART.println();
    debugUART.println("RUNNING!");  
    debugUART.println(); 

    mutex_v = xSemaphoreCreateMutex();  //The mutex we will use to protect the jsonMsgBuffer
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

  //setNormalPowerMode();
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
  http.begin(params.serverURL.c_str());
  
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
    jsonHeader = "{\"deviceName\":\"" + params.deviceName + 
                   "\",\"deviceMAC\":\"" + getMACAddress() + 
                   "\",\"timeStamp\":\"" + getRTCTime() + 
                   "\",\"eventType\":\"" + eventType + 
                   "\"";
  } else {
    jsonHeader = "{\"deviceName\":\"" + params.deviceName + 
                   "\",\"deviceMAC\":\"" + getMACAddress() + 
                   "\",\"timeStamp\":\"" + getLocalTime() + 
                   "\",\"eventType\":\"" + eventType + 
                   "\"";  
  }                    
  return jsonHeader;
}

//****************************************************************************************
// LoRa messages to the server all have a similar format. 
#if USE_LORA
String buildLoRaHeader(String eventType, double count){
  String loraHeader;
  String strCount;

  strCount= String(count,0);
  strCount.trim();
  if (rtcPresent){
    loraHeader = "{\"ts\":\"" + getRTCTime() + 
                   "\",\"et\":\"" + eventType +
                   "\",\"c\":\"" + strCount +  
                   "\"";
  } else {
    loraHeader = "{\"ts\":\"" + getLocalTime() + 
                   "\",\"et\":\"" + eventType + 
                   "\",\"c\":\"" + strCount + 
                   "\"";  
  }                    
  return loraHeader;
}
#endif


//************************************************************************
//We have a JSON message to send to the server. Try to send it. If it fails,
//Save it to the SD card for later delivery when we can reconnect.
void processMessage(String jsonPayload){

#if USE_WIFI
  wakeModemSleep();
  if(WiFi.status()== WL_CONNECTED){
      // We have a WiFi connection. -- Upload the data to the the server. 
      postJSON(jsonPayload);
      
  } /*else {
      // No WiFi -- Save locally.
      appendDatalog(jsonPayload);
  
      // Try connecting every five minutes 
      if ((millis() - msLastConnectionAttempt) > (5*60*1000)){ 
        //Try to connect up to two times...
        connectToWiFi(params.networkName, params.password);    // Once
        
        if (WiFi.status() != WL_CONNECTED){
          connectToWiFi(params.networkName, params.password);  // Twice
        }

        msLastConnectionAttempt=millis();
        
        if (WiFi.status() == WL_CONNECTED){
          postDatalog();  // POSTS and clears out the log.
        }
     }
   }*/
  setModemSleep();  
#endif

}


//************************************************************************
// LIDAR Siganal Processing Functions
//************************************************************************

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

    //tfmP.sendCommand(TRIGGER_DETECTION, 0); //Uncomment in triggered mode.
    delay(lidarUpdateRate);

    // Read the LIDAR Sensor
    if( tfmP.getData( tfDist, tfFlux, tfTemp)) { 

      // When very close, or looking off into empty space, the sensor reports Zero.
      // The short range isn't an issue for us. 
      // Any Zeros we see will be due to no reflective target in range.
      // Assume max range for the sensor to get the threshold algorithm to perform properly. 
      if (tfDist == 0){ 
        tfDist = 1200;  
      }
      
      //Filter the measured distance
      smoothed = smoothed *0.65 + (float)tfDist * 0.35;
      int intSmoothed = (int) smoothed*10;

      buffer.push(intSmoothed);
      
      if (smoothed < params.distanceThreshold) {
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
     debugUART.print(params.distanceThreshold);
     debugUART.print(",");
     debugUART.println(carEvent);
#endif
     
     lastCarPresent = carPresent;
  }

    
  return retValue;  
}



// Sends a message to another LoRa module and listens for an ACK reply.
bool sendReceiveLoRa(String msg){
  long timeout = 10000;
  long t2,t1;

  t1 = millis();
  t2 = t1;

  bool replyPending = true;
  
  //Send the message... Base Stations Default to an address of 1.
  String reyaxMsg = "AT+SEND=1,"+String(msg.length())+","+msg;
  debugUART.print("Sending LoRa Message: ");
  debugUART.println(reyaxMsg);
  LoRaUART.println(reyaxMsg);
  
  //wait for ACK or timeout
  while ((replyPending == true) && ((t2-t1)<timeout)){
    t2=millis();
    if (LoRaUART.available()) {
      String inString = LoRaUART.readStringUntil('\n');
      if (replyPending) {
        if (inString.indexOf("ACK")>=0){
          replyPending = false;
          debugUART.println("ACK Received: " + inString);
          return true;
          
        }
      } 
    } 
  }
  
  if((t2-t1)>= timeout){
    debugUART.println("Timeout!");
    return false;
  }

  vTaskDelay(1000 / portTICK_PERIOD_MS); 
  
}




/********************************************************************************/
// Experimenting with using a circular buffer to enqueue messages to the server...
void messageManager(void *parameter){
  String activeMessage;
  
  for(;;){  

    #if USE_LORA
      //****************************
      //LoRa message
      //****************************
      if (loraMsgBuffer.size()>0){
        #if SHOW_DATA_STREAM
        #else
          Serial.print("MessageManager: loraMsgBuffer.size(): ");
          Serial.println(loraMsgBuffer.size());
          Serial.println("MessageManager: Sending LoRa message: ");
        #endif  
            
        xSemaphoreTake(mutex_v, portMAX_DELAY); 
          activeMessage=loraMsgBuffer.shift();
        xSemaphoreGive(mutex_v);
        
        #if SHOW_DATA_STREAM
        #else
         Serial.println(activeMessage);
        #endif
  
        //Wake up the LoRa module
        //LoRaUART.println("AT");
        //vTaskDelay(500 / portTICK_PERIOD_MS);

        //Send the data to the base station

        
        while (!sendReceiveLoRa(activeMessage)){};
        
        //LoRaUART.print("AT+SEND=1,");
        //LoRaUART.println(String(loraPayload.length()) + "," + activeMessage); 
        //delay(3000);

        
        
        //Put LoRa module to sleep
        //LoRaUART.println("AT+MODE=1");
        //vTaskDelay(500 / portTICK_PERIOD_MS);
             
        Serial.println("Sent!");
      } else{
        //Serial.println("MessageManager: Nothing to do...");      
      }
    #endif


    #if USE_WIFI
      //****************************
      //JSON message
      //****************************
      if (jsonMsgBuffer.size()>0){
  
        #if SHOW_DATA_STREAM
        #else
          Serial.print("MessageManager: jsonMsgBuffer.size(): ");
          Serial.println(jsonMsgBuffer.size());
          Serial.println("MessageManager: Sending message: ");
        #endif  
            
        xSemaphoreTake(mutex_v, portMAX_DELAY); 
          activeMessage=jsonMsgBuffer.shift();
        xSemaphoreGive(mutex_v);
        
       #if SHOW_DATA_STREAM
       #else
         Serial.println(activeMessage);
       #endif
        
        processMessage(activeMessage);        
      } else{
        //Serial.println("MessageManager: Nothing to do...");  
      }
    #endif 

    vTaskDelay(100 / portTICK_PERIOD_MS);   
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
    if (params.detAlgorithm=="Correlation"){      
        vehicleMessageNeeded = processLIDARSignalCorrel(); //Correlation Detection
    } else{ 
      if (params.detAlgorithm=="Threshold"){
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
      #if USE_WIFI
        jsonPayload = buildJSONHeader("boot");
        jsonPayload = jsonPayload + "}";
      #endif
      
      #if USE_LORA
        loraPayload = buildLoRaHeader("b",count);
        loraPayload = loraPayload + "}";
      #endif
      
      jsonPostNeeded = true;
      bootMessageNeeded = false;
    }

    // Issue a heartbeat message every hour.
    heartbeatTime = getRTCMinute();
    if ((oldHeartbeatTime != bootMinute) && (heartbeatTime == bootMinute)){heartbeatMessageNeeded = true;}  
    
    if (heartbeatMessageNeeded){
      
      #if USE_WIFI
        jsonPayload = buildJSONHeader("heartbeat");
        jsonPayload = jsonPayload + "}";
      #endif 
      
      #if USE_LORA
        loraPayload = buildLoRaHeader("hb",count);
        loraPayload = loraPayload + "}";
      #endif
  
      jsonPostNeeded = true;
      heartbeatMessageNeeded = false;
    }
    oldHeartbeatTime = heartbeatTime;
    
    if (vehicleMessageNeeded){
      
      if (buffer.size()==samples){ //Fill up the buffer before processing so we don't get false events at startup.
        
        count++;
        
        #if SHOW_DATA_STREAM
        #else
          debugUART.print("Loop: Count event. Counts: ");
          debugUART.println(count);  
        #endif 
             
        #if USE_EINK
          showValue(count);
        #endif

        #if USE_WIFI
          jsonPayload = buildJSONHeader("vehicle");
          jsonPayload = jsonPayload + ",\"linkMode\":\"" + params.linkMode +"\"";
          jsonPayload = jsonPayload + ",\"detAlgorithm\":\"" + params.detAlgorithm +"\"";
          
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
          jsonPayload = jsonPayload + "]";
          #endif
          
          jsonPayload = jsonPayload + "}";
        #endif
        
        #if USE_LORA
          loraPayload = buildLoRaHeader("v",count);
          // Detection algorithm
          String da = String(params.detAlgorithm[0]); //[T]hreshold or [C]orrelation
          da.toLowerCase();
          loraPayload = loraPayload + ",\"da\":\"" + da +"\"";
          loraPayload = loraPayload + "}";
        #endif
        
        jsonPostNeeded = true; 
      }
      vehicleMessageNeeded = false; 
    }
     
    // Push the JSON Payload into the message buffer. The messageManager task will handle it. 
    if (jsonPostNeeded){
      
      xSemaphoreTake(mutex_v, portMAX_DELAY);  
        #if USE_WIFI       
          jsonMsgBuffer.push(jsonPayload); 
        #endif
        
        #if USE_LORA
          loraMsgBuffer.push(loraPayload);
        #endif
      xSemaphoreGive(mutex_v);
      
      //processMessage(jsonPayload); 
      jsonPostNeeded = false; 
      
      if (params.detAlgorithm=="correlation"){ 
        buffer.clear();  // TODO: think about a way to do the POST in a separate thread so we can continuously gather data...
      }
      
    }   
}
