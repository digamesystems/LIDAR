/*  A simple LoRa base station that listens for events from our Vehicle Counters. 
 *   
 *  Build up a JSON message for each event and route to our server over WiFi.
 *  
 *  Copyright 2021, Digame systems. All rights reserved. 
 */
 
// Mix-n-match hardware configuration
#define USE_WIFI true           // WiFi back haul
#define USE_LORA true           // Reyax896 LoRa Module back haul
#define USE_EINK true           // Adafruit eInk Display
#define ADAFRUIT_EINK_SD true   // Some Adafruit Eink Displays have an integrated SD card so we don't need a separate module
 
#include <WiFi.h>               // WiFi stack

#if USE_WIFI
  #include <HTTPClient.h>       // To post to the ParkData Server
#endif

#include <CircularBuffer.h>     // Adafruit library. Pretty small!

#include "digameTime.h"         // Digame Time Functions
#include "digameJSONConfig.h"   // Program parameters from config file on SD card
#include "digameNetwork.h"      // Digame Network Functions

#if USE_EINK
  #include "digameDisplay.h"    // Digame eInk Display Functions.
#endif

//#include "driver/adc.h"
//#include <esp_bt.h>
#include <ArduinoJson.h>

#define loraUART Serial1
#define debugUART Serial

int RX_LED = 13;
int LED_DIAG = 12;  // Indicator LED
int CTR_RESET = 32; // Counter Reset Input


long   msLastConnectionAttempt=0;
bool   wifiConnected = false;
bool rtcPresent    = false;


bool   loraConfigMode = false;
String cmdMsg;
String loraMsg;

SemaphoreHandle_t mutex_v; // Mutex used to protect our jsonMsgBuffers (see below).

const int samples = 20;
CircularBuffer<String, samples> loraMsgBuffer; // A buffer containing JSON messages sent to the LoRa basestation.

void setLowPowerMode();
void setNormalMode();
void enableWiFi();
 

//************************************************************************
void initPorts(){
  pinMode(RX_LED, OUTPUT);
  pinMode(CTR_RESET,INPUT_PULLUP);
  loraUART.begin(115200, SERIAL_8N1, 25,33);
  debugUART.begin(115200); 
  delay(1000);
  Wire.begin();
}


//************************************************************************
void splash(){
  String compileDate = F(__DATE__);
  String compileTime = F(__TIME__);

  debugUART.println("*****************************************************");
  debugUART.println("HEIMDALL VCS - LoRa Base Station");
  debugUART.println("Version 0.9.3");
  debugUART.println("Copyright 2021, Digame Systems. All rights reserved.");
  debugUART.println();
  debugUART.print("Compiled on ");
  debugUART.print(compileDate);
  debugUART.print(" at ");
  debugUART.println(compileTime); 
  debugUART.println("*****************************************************");
  debugUART.println();

  display.init(0);
  // first update should be full refresh
  initDisplay();
  displaySplashScreen();
  //delay(1000);
  //displayInitializingScreen();
}


//************************************************************************
// Save a single JSON message to the server.
void postJSON(String jsonPayload, String strServerURL){

  if (WiFi.status() != WL_CONNECTED){
    debugUART.println("WiFi Connection Lost.");
    enableWiFi();      
  }

  HTTPClient http;
  
  // Your Domain name with URL path or IP address with path
  http.begin(strServerURL);
  
  // If you need an HTTP request with a content type: application/json, use the following:
  http.addHeader("Content-Type", "application/json");

 
  int httpResponseCode = http.POST(jsonPayload);

  debugUART.print("HTTP response code: ");
  debugUART.println(httpResponseCode);

  // Free resources
  http.end();
  
  return;
}


//************************************************************************
void processLoRaMessage(String msg){

  StaticJsonDocument<512> doc;

  // Start and end of the JSON payload in the msg.
  int idxstart = msg.indexOf('{');
  int idxstop = msg.indexOf('}')+1;

  char json[512] = {};

  // The message contains a JSON payload extract to the char array json
  String payload = msg.substring(idxstart,idxstop); 
  payload.toCharArray(json,payload.length()+1);

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, json);

  // Test if parsing succeeds.
  if (error) {
    debugUART.print(F("deserializeJson() failed: "));
    debugUART.println(error.f_str());
    return;
  }

  // After the payload comes the RSSI and SNR values;
  String trailer = msg.substring(idxstop +1);
  //debugUART.println(trailer);
  idxstop = trailer.indexOf(',');
  
  String strRSSI = trailer.substring(0,idxstop);
  //debugUART.println(strRSSI);
  strRSSI.trim();

  String strSNR = trailer.substring(idxstop + 1);
  //debugUART.println(strSNR);  
  strSNR.trim();
  
  // Fetch values.
  //
  // Most of the time, you can rely on the implicit casts.
  // In other case, you can do doc["time"].as<long>();
  String strEventType;
  String et = doc["et"];
   
  if (et=="b"){
      strEventType = "Boot";
  } else if (et == "h"){
      strEventType = "Heartbeat";
  } else if (et == "v"){
      strEventType = "Vehicle";
  } else { 
      strEventType = "Unknown";
  } 

  // Timestamp
  String strTime = doc["ts"];
 
  // Count
  String strCount = doc["c"];
 
  // Detection Algorithm
  String strDetAlg;
  String da = doc["da"];

  if (da == "t"){
    strDetAlg = "Threshold";
  } else if (da == "c") {
    strDetAlg = "Correlation";
  } else {
    strDetAlg = "Unknown";
  }

  String jsonPayload;

  // TODO: keep a list of Vehicle Counters we know and get this from a boot message.
  String strDeviceName = "Tucson Base Station";
  String strDeviceMAC  = "00:01:02:03:04:05";

  jsonPayload = "{\"deviceName\":\""       + strDeviceName + 
                 "\",\"deviceMAC\":\""     + strDeviceMAC  + 
                 "\",\"timeStamp\":\""     + strTime + 
                 "\",\"linkMode\":\""      + "LoRa" +
                 "\",\"eventType\":\""     + strEventType + 
                 "\",\"detAlgorithm\":\""  + strDetAlg +
                 "\",\"count\":\""         + strCount + 
                 "\",\"rssi\":\""          + strRSSI + 
                 "\",\"snr\":\""           + strSNR +              
                 "\"}";

  //debugUART.println(jsonPayload);
  postJSON(jsonPayload, config.serverURL);

}


//************************************************************************
void blinkLED(){
  digitalWrite(RX_LED, HIGH);   
  delay(100);
  digitalWrite(RX_LED, LOW);  
}


//************************************************************************
void enableWiFi(){
    //adc_power_on();
    delay(200);
 
    WiFi.disconnect(false);  // Reconnect the network
    WiFi.mode(WIFI_OFF);     // Switch WiFi off
 
    delay(200);
 
    debugUART.print("Starting WiFi");
    WiFi.begin(config.ssid.c_str(), config.password.c_str());
 
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        debugUART.print(".");
    }
 
    debugUART.println("");
    debugUART.println("  WiFi connected.");
    debugUART.print("  IP address: ");
    debugUART.println(WiFi.localIP());
    wifiConnected = true;
    
    
}


//************************************************************************
void disableWiFi(){
    //adc_power_off();
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF);    // Switch WiFi off
    debugUART.println("");
    debugUART.println("WiFi disconnected!");
}


//************************************************************************
void disableBluetooth(){
    // Quite unuseful, no real savings in power consumption
    btStop();
    debugUART.println("");
    debugUART.println("Bluetooth stoppedz!");
}


//************************************************************************
void setLowPowerMode() {
    debugUART.println("Adjusting Power Mode:");
    disableWiFi();
    disableBluetooth();
    setCpuFrequencyMhz(40);
    
    //Set LoRa module into sleep mode.
    loraUART.println("AT+MODE=1");
    debugUART.println("  Low Power Mode Enabled.");
    
    // Use this if 40Mhz is not supported
    // setCpuFrequencyMhz(80);
}
 
//************************************************************************ 
void setNormalMode() {
    //setCpuFrequencyMhz(240);
    enableWiFi(); 
    //debugUART.println("Adjusting Power Mode:");
    // Wake up LoRa module.
    loraUART.println("AT");   
    //debugUART.println("  Normal Mode Enabled.");
}


/********************************************************************************/
// Experimenting with using a circular buffer and multi-tasking to enqueue 
// messages to the server...
void messageManager(void *parameter){
  String activeMessage;

  for(;;){  

    //Serial.println("messageManager TICK");
    
    //********************************************
    // Check for Messages on the Queue and Process
    //********************************************
    if (loraMsgBuffer.size()>0){

      Serial.print("MessageManager: loraMsgBuffer.size(): ");
      Serial.println(loraMsgBuffer.size());
      Serial.println("MessageManager: Processing LoRa message: ");

      xSemaphoreTake(mutex_v, portMAX_DELAY); 
        activeMessage=loraMsgBuffer.shift();
      xSemaphoreGive(mutex_v);
      
      Serial.println(activeMessage);

      processLoRaMessage(activeMessage);

    } else{
      //Serial.println("MessageManager: Nothing to do...");      
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);   
  }   
}

//************************************************************************
// Setup
//************************************************************************
void setup() {
  initPorts(); //Set up serial ports and GPIO
  
  splash();
    
  debugUART.println("INITIALIZING\n");

  initSDCard();
  debugUART.println("Reading Parameters from SD Card...");
  loadConfiguration(filename,config);
  debugUART.println("  Device Name: " + config.deviceName);
  debugUART.println("  SSID: " + config.ssid);
  debugUART.println("  ServerURL: " + config.serverURL);
  
  //printFile(filename);  
  
  setNormalMode(); //Run at full power and max speed with WiFi enabled by default
  debugUART.println("  MAC Address: " + getMACAddress());  

  debugUART.println("Setting LoRa Device Address...");
  loraUART.println("AT+ADDRESS=1"); //MY ADDRESS -- Base Stations Default to 1.
  delay(1000);
  debugUART.println("  Address = 1");


  debugUART.println("Testing for Real-Time-Clock module... ");
  rtcPresent = initRTC();
  if (rtcPresent){
    debugUART.println("  RTC found. (Program will use time from RTC.)");
    //stat +="   RTC  : OK\n";
    debugUART.print("    RTC Time: ");
    debugUART.println(getRTCTime()); 
  }else{
    debugUART.println("ERROR! Could NOT find RTC. (Program will attempt to use NTP time.)");   
    //stat +="    RTC  : ERROR!\n"; 
  }
  
  if (wifiConnected){ // Attempt to synch ESP32 clock with NTP Server...
    synchTime(rtcPresent);
  }
    

  mutex_v = xSemaphoreCreateMutex();  //The mutex we will use to protect the jsonMsgBuffer
  
  xTaskCreate(
    messageManager,    // Function that should be called
    "Message Manager",   // Name of the task (for debugging)
    7000,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1,               // Task priority
    NULL             // Task handle
  );

 
  debugUART.println();
  debugUART.println("RUNNING\n");
}


//************************************************************************
// Main Loop
//************************************************************************
void loop() {

  // Handle what the LoRa module has to say. 
  // If it's a message from another module, add it to the queue
  // so the manager function can handle it.
  //
  // Otherwise, just echo to the debugUART.
  
  if (loraUART.available()) {
    
    loraMsg = loraUART.readStringUntil('\n');
    
    //Messages received by the Module start with '+RCV'
    if (loraMsg.indexOf("+RCV")>=0){
      debugUART.println();
      debugUART.print("LoRa Message Received. ");  
      debugUART.println(loraMsg);

      // Send an acknowlegement to the sender.
      //Grab the address of the sender
      // Messages are of the form: "+RCV=2,2,16,-64,36" -- where the first number after the "=" is the address.
      
      // Start and end of the JSON payload in the msg.
       int idxstart = loraMsg.indexOf('=')+1;
       int idxstop = loraMsg.indexOf(',');
    
      // grab the address
      String senderAddress = loraMsg.substring(idxstart,idxstop); 
    
      // Now we can answer anyone who talks to us.
      debugUART.print("Sending ACK: ");
      loraUART.println("AT+SEND=" + senderAddress + ",3,ACK"); 

      xSemaphoreTake(mutex_v, portMAX_DELAY);      
        loraMsgBuffer.push(loraMsg);
        debugUART.print("loraMsgBuffer.size: ");
        debugUART.println(loraMsgBuffer.size());
        debugUART.println();
      xSemaphoreGive(mutex_v);
  


    } else {
      debugUART.println(loraMsg);      
    }
         
  }
  
  // Someone is sending us data on the debugUART. 
  // Only route this to the LoRa module if we are in loraConfigMode.
  
  if (debugUART.available()) {
    cmdMsg = debugUART.readStringUntil('\n');    
    cmdMsg.trim();
    if (cmdMsg.indexOf("CONFIG")>=0){ 
      debugUART.println("Entering LoRa configuration mode:");
      loraConfigMode=true;
      cmdMsg = "AT"; //Get the module's attention.
    }
    if (cmdMsg.indexOf("EXIT")>=0) {
      debugUART.println("Leaving LoRa configuration mode:");
      loraConfigMode=false;
    }  
    
    // If in config mode, route serial monitor message to the module for config, etc. 
    if (loraConfigMode) {
      loraUART.print(cmdMsg + "\r\n");
    } else {
      // use the debugUART message for other purposes, e.g., menu system, etc.   
      if (cmdMsg.indexOf("SLEEP")>=0) {
        setLowPowerMode();
      }
      if (cmdMsg.indexOf("WAKE")>=0) {
        setNormalMode();
      }
    } 
    
  } 
}
