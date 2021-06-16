/*  A simple LoRa base station that listens for events from our Vehicle Counters. 
 *   
 *  Build up a JSON message for each event and route to our server over WiFi.
 *  
 *  Copyright 2021, Digame systems. All rights reserved. 
 */
 
#define USE_WIFI true

#include "digameNetwork.h"
#include <HTTPClient.h>      // To post to the ParkData Server
#include "driver/adc.h"
#include <esp_bt.h>

#include <ArduinoJson.h>

#define loraUART Serial1
#define debugUART Serial

int    RX_LED = 13;

//TODO: Read these from memory or SD card.
String strSSID        = "AndroidAP3AE2";
String strPassword    = "ohpp8971";
String strServerURL   = "https://trailwaze.info/zion/lidar_sensor_import.php";


long   msLastConnectionAttempt=0;
bool   wifiConnected = false;

bool   loraConfigMode = false;
String cmdMsg;
String loraMsg;

void setModemSleep();
void wakeModemSleep();
 

//************************************************************************
void initHardware(){
  pinMode(RX_LED, OUTPUT);
  loraUART.begin(115200, SERIAL_8N1, 25,33);
  debugUART.begin(115200); 
  delay(1000);
}


//************************************************************************
void splash(){
  String compileDate = F(__DATE__);
  String compileTime = F(__TIME__);

  debugUART.println("*****************************************************");
  debugUART.println("LoRa Base Station");
  debugUART.println("Version 0.9");
  debugUART.println("Copyright 2021, Digame Systems. All rights reserved.");
  debugUART.println();
  debugUART.print("Compiled on ");
  debugUART.print(compileDate);
  debugUART.print(" at ");
  debugUART.println(compileTime); 
  debugUART.println("*****************************************************");
  debugUART.println();
}


/************************************************************************
void initWiFi(){
  debugUART.print("  Testing for WiFi connectivity... ");
  
  //wifiConnected = initWiFi("AndroidAP3AE2", "ohpp8971"); //My phone hotspot
  wifiConnected = initWiFi(strSSID, strPassword);
  
  msLastConnectionAttempt=millis();  // Save the time of the latest connection attempt. Used to schedule retries if lost.
  
  debugUART.println();
  if (wifiConnected){
    debugUART.println("      Connection established.");
  }else{
    debugUART.println("      ERROR! Could NOT establish WiFi connection. (Credentials OK?)");  
  }
  
  debugUART.print("  Reading Device MAC Address... ");
  debugUART.println(getMACAddress());
  debugUART.println(); 
}
*/

//************************************************************************
// Save a single JSON message to the server.
void postJSON(String jsonPayload, String strServerURL){

#if USE_WIFI
  HTTPClient http;
  
  // Your Domain name with URL path or IP address with path
  http.begin(strServerURL);
  
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

  String strDeviceName = "Digame LoRa Base Station";
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

  debugUART.println(jsonPayload);
  postJSON(jsonPayload, strServerURL);

}


//************************************************************************
void blinkLED(){
  digitalWrite(RX_LED, HIGH);   
  delay(100);
  digitalWrite(RX_LED, LOW);  
}


void disableWiFi(){
    adc_power_off();
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF);    // Switch WiFi off
    debugUART.println("");
    debugUART.println("WiFi disconnected!");
}
void disableBluetooth(){
    // Quite unuseful, no real savings in power consumption
    btStop();
    debugUART.println("");
    debugUART.println("Bluetooth stop!");
}
 
void setModemSleep() {
    disableWiFi();
    disableBluetooth();
    setCpuFrequencyMhz(40);
    
    //Set LoRa module into sleep mode.
    loraUART.println("AT+MODE=1");
    debugUART.println("MODEM SLEEP ENABLED");
    
    // Use this if 40Mhz is not supported
    // setCpuFrequencyMhz(80);
}
 
void enableWiFi(){
    adc_power_on();
    delay(200);
 
    WiFi.disconnect(false);  // Reconnect the network
    WiFi.mode(WIFI_STA);    // Switch WiFi off
 
    delay(200);
 
    debugUART.println("START WIFI");
    WiFi.begin(strSSID.c_str(), strPassword.c_str());
 
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        debugUART.print(".");
    }
 
    debugUART.println("");
    debugUART.println("WiFi connected");
    debugUART.println("IP address: ");
    debugUART.println(WiFi.localIP());
    
}
 
void wakeModemSleep() {
    setCpuFrequencyMhz(240);
    enableWiFi(); 
    
    // Wake up LoRa module.
    loraUART.println("AT");   
    debugUART.println("MODEM SLEEP DISABLED.");
}


//************************************************************************
// Setup
//************************************************************************
void setup() {
  initHardware();  
  splash();
  debugUART.println("INITIALIZING...\n");
  //initWiFi();  
  wakeModemSleep();
    
  debugUART.println("RUNNING...\n");
  //setModemSleep();

}



unsigned long t1 = millis();
unsigned long t2 = t1;
bool started = false;

//************************************************************************
// Main Loop
//************************************************************************
void loop() {

  // Handle what the LoRa module has to say. 
  // If it's a message from another module, run it through the processing 
  // routine. 
  //
  // Otherwise, just echo to the debugUART.
  
  if (loraUART.available()) {
    
    loraMsg = loraUART.readStringUntil('\n');
    debugUART.println(loraMsg);
    
    if (loraMsg.indexOf("+RCV")>=0){
      // Send an acknowlegement to the sender.
      // TODO: No hardcoding of sender address!      
      loraUART.println("AT+SEND=1,3,ACK");

      blinkLED();
      
      processLoRaMessage(loraMsg);
      debugUART.print("LoRa Message Received: ");   
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
    
    
    if (loraConfigMode) {
      loraUART.print(cmdMsg + "\r\n");
    } else {
      // use the debugUART message for other purposes, e.g., menu system, etc.   
      if (cmdMsg.indexOf("SLEEP")>=0) {
        setModemSleep();
      }
      if (cmdMsg.indexOf("WAKE")>=0) {
        wakeModemSleep();
      }
    } 
    
  } 
}
