/* esp_lora_basestation
 *  
 *  A simple LoRa listener for events from our Vehicle Counters. 
 *  Builds up a JSON message for each event and routes them our server 
 *  over WiFi.
 *  
 *  The program provides a web interface for configuration of LoRa
 *  and WiFi parameters.
 *  
 *  Copyright 2021, Digame systems. All rights reserved. 
 */
 

#define USE_LORA true

//for Over-the-Air updates...
#include <WiFi.h>
#include <ArduinoOTA.h>

#include <digameVersion.h>

#include <CircularBuffer.h>   // Adafruit library. Pretty small!
#include <ArduinoJson.h>      // 

#include <digameJSONConfig.h> // Program parameters from config file on SD card
#include <digameTime.h>       // Time Functions - RTC, NTP Synch etc
#include <digameNetwork.h>    // Network Functions - Login, MAC addr
#include <digameWebServer.h>  // Handles parmater tweaks through a web page
#include <digamePowerMgt.h>   // Power management modes 
#include <digameDisplay.h>    // eInk Display Functions
#include <digameLoRa.h>       // Reyax LoRa module control functions

#define debugUART Serial
#define CTR_RESET 32          // Reset Button Input


// GLOBALS

Config config; // Program configuration variables. See: digameJSONConfig.h
               // This variable is used all over the place. 

int displayMode = 1; // Which eInk screen we are showing: 1=Title, 2=Network, 3=Last Message

// Our network name when we are running in Access Point Mode.
const char *ssid = "Digame-STN-AP"; //STN = "(Base) Station"
bool   accessPointMode = false;

// Heartbeat managment variables
int    heartbeatMinute;    // Issue Heartbeat message once an hour. Holds the current Minute.
int    oldheartbeatMinute; // Value the last time we looked.
int    bootMinute;         // The minute (0-59) within the hour we woke up at boot. 
bool   heartbeatMessageNeeded = true;
bool   bootMessageNeeded = true;

String currentTime;  // Update in Main Loop
String myMACAddress; // Grab at boot time

// Multi-Tasking
SemaphoreHandle_t mutex_v; // Mutex used to protect variables across RTOS tasks. 
TaskHandle_t messageManagerTask;
TaskHandle_t eventDisplayManagerTask;

String strDisplay=""; // contents of the event screen.

// Counter values for each counter
String str1Count = "0";
String str2Count = "0"; 
String str3Count = "0"; 
String str4Count = "0"; 
String strTotal  = "0";
    
const int samples = 200;
CircularBuffer<String, samples> loraMsgBuffer; // A buffer containing JSON messages to be 
                                               //   sent to the Server

// FUNCTION DECLARATIONS

void   initPorts();
void   splash();
String buildJSONHeader(String);
void   processLoRaMessage(String);
void   messageManager(void *);


//****************************************************************************************
// Configure the IO pins and set up the UARTs
void initPorts(){
  
  pinMode(CTR_RESET,INPUT_PULLUP);
  initLoRa();
  debugUART.begin(115200); 
  delay(1000);
  Wire.begin();

}


//****************************************************************************************
void splash(){
  
  String compileDate = F(__DATE__);
  String compileTime = F(__TIME__);

  debugUART.println("*****************************************************");
  debugUART.println("         HEIMDALL Vehicle Counting System");
  debugUART.println("              - BASE STATION UNIT -");
  debugUART.println();
  debugUART.println("Model: DS-VC-BASE-LOR-1 (WiFi/LoRa Base Station)");
  debugUART.println("Version: " + SW_VERSION);
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

  // first update should be full refresh
  initDisplay();
  displaySplashScreen("(Base Station)",SW_VERSION);

}

//****************************************************************************************
// Save a JSON log file to the server
void postDatalog(){
  if (initSDCard()){
    String jsonPayload="";

    debugUART.println("POSTING DATALOG.TXT to server...");
    
    File dataFile = SD.open("/DATALOG.TXT");
  
    // If the file is available, read from it:
    if (dataFile) {
       while (dataFile.available()) {
          jsonPayload = dataFile.readStringUntil('\r');
          jsonPayload.trim();
          postJSON(jsonPayload, config);
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


//*****************************************************************************
// Append a message to the DATALOG.TXT file on the SD card.
// TODO: Error handling if card is full, etc.
void appendDatalog(String jsonPayload){
  if (initSDCard()){
    File dataFile = SD.open("/DATALOG.TXT", FILE_APPEND);
  
    // If the file is available, write to it:
    if (dataFile) {
      debugUART.println("Appending data file on SD:");
      debugUART.println(jsonPayload);
      
      dataFile.println(jsonPayload);
      dataFile.close();
    }
    // If the file isn't open, pop up an error:
    else {
      debugUART.println("ERROR! Trouble opening datalog.txt");
    }
  } else {
    Serial.println("ERROR! SD Card not present.");  
  }
}



//****************************************************************************************
String loraMsgToJSON(String msg){
  static String lastMessageCountValue;

  StaticJsonDocument<512> doc;

  // Get the device's Address.
  int idxstart = msg.indexOf('=')+1;
  int idxstop  = msg.indexOf(','); 
  String strAddress = msg.substring(idxstart,idxstop);
  //debugUART.println(strAddress);
  
  // Start and end of the JSON payload in the msg.
  idxstart = msg.indexOf('{');
  
  //idxstop = msg.indexOf('}')+1; // Changed the approach after we started nesting JSON structs. 
    idxstop = msg.length() - 8;   // Now going from the end of the msg back 8 chars: 
                                  // ",-XX,YY" (XX =RSSI, YY=SNR) 
  
  char json[512] = {};

  // The message contains a JSON payload extract to the char array json
  String payload = msg.substring(idxstart,idxstop); 
  //debugUART.println("LORA Payload");
  //debugUART.println(payload);
  payload.toCharArray(json,payload.length()+1);

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, json);

  // Test if parsing succeeds.
  if (error) {
    debugUART.print(F("deserializeJson() failed: "));
    debugUART.println(error.f_str());
    return "IGNORE";
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

  String strVersion = doc["v"];
  
  if (et=="b"){
      strEventType = "Boot";
  } else if (et == "hb"){
      strEventType = "Heartbeat";
  } else if (et == "v"){
      strEventType = "Vehicle";
  } else { 
      strEventType = "Unknown";
      return "IGNORE";  // If we don't know what this is, don't bother the server with it.
  } 

  // Timestamp
  String strTime = doc["ts"];
   
  // Count
  String strCount = doc["c"];
  if (lastMessageCountValue.equals(strCount)) {
    debugUART.println("We've seen this message before.");
    return "IGNORE";  
  }
  lastMessageCountValue = strCount;
 
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

  String strTemperature = doc["t"];

  String jsonPayload;

  String strDeviceName = "Unknown Device";
  String strDeviceMAC  = "00:01:02:03:04:05";

  String strSettings = doc["s"];

  // TODO: move to a look up function and come up with a better storage scheme.
  if (strAddress == config.sens1Addr){
    strDeviceName = config.sens1Name;
    strDeviceMAC = config.sens1MAC;    
    str1Count = strCount;
  } else if (strAddress == config.sens2Addr){
    strDeviceName = config.sens2Name;
    strDeviceMAC = config.sens2MAC;
    str2Count = strCount;
  } else if (strAddress == config.sens3Addr){
    strDeviceName = config.sens3Name;
    strDeviceMAC = config.sens3MAC;
    str3Count = strCount;
  } else if (strAddress == config.sens4Addr){
    strDeviceName = config.sens4Name;
    strDeviceMAC = config.sens4MAC;
    str4Count = strCount;
  }

  strTotal = String( str1Count.toInt() + str2Count.toInt() + str3Count.toInt() + str4Count.toInt());

  

  String strRetries = doc["r"];
  
  if (displayMode ==3) { // Update the eInk display with the latest information
    strDisplay = "Addr: " + strAddress + 
                 "\nEvent:" + strEventType + 
                 "\nCount:" + strCount + 
                 "\nTemp: " + strTemperature +
                 "\nDate: " + strTime.substring(0,strTime.indexOf(" ")) +
                 "\nTime: " + strTime.substring(strTime.indexOf(" ")+1);    
  }

  jsonPayload = "{\"deviceName\":\""       + strDeviceName + 
                 "\",\"deviceMAC\":\""     + strDeviceMAC  + 
                 "\",\"firmwareVER\":\""   + strVersion  + 
                 "\",\"timeStamp\":\""     + strTime + 
                 "\",\"linkMode\":\""      + "LoRa" +
                 "\",\"eventType\":\""     + strEventType + 
                 "\",\"detAlgorithm\":\""  + strDetAlg +
                 "\",\"count\":\""         + strCount + 
                 "\",\"rssi\":\""          + strRSSI + 
                 "\",\"snr\":\""           + strSNR +    
                 "\",\"temp\":\""          + strTemperature +  
                 "\",\"retries\":\""       + strRetries; 
                 
 if ((et=="b")||("et"=="hb")){
  jsonPayload = jsonPayload + "\",\"settings\":" + strSettings;                  
  jsonPayload = jsonPayload + "}";
 } else {       
  jsonPayload = jsonPayload + "\"}";
 }
  return jsonPayload;
  
}

//****************************************************************************************
// Parse a LoRa message from a vehicle counter. Format as a JSON message
// and POST it to the server if WiFi is available. If not, save to the SD card. 
void processLoRaMessage(String msg){
  String jsonPayload;

  jsonPayload = loraMsgToJSON(msg);

  if (jsonPayload == "IGNORE"){
    return;
  }
  
  if(WiFi.status()== WL_CONNECTED){
      // We have a WiFi connection. -- Upload the data to the the server. 
      postJSON(jsonPayload, config);
      
  } else {
      // No WiFi -- Save locally.
      appendDatalog(jsonPayload);
  
      // Try connecting every five minutes 
      if ((millis() - msLastConnectionAttempt) > (5*60*1000)){ 

        enableWiFi(config);
   
        if (WiFi.status() == WL_CONNECTED){
          postDatalog();  // POSTS and clears out the log.
        }
     }
   } 
}


//****************************************************************************************
// JSON messages to the server all have a similar format. 
String buildJSONHeader(String eventType){
  String jsonHeader;

  jsonHeader = "{\"deviceName\":\""      + config.deviceName + 
                 "\",\"deviceMAC\":\""   + myMACAddress + // Read at boot
                 "\",\"firmwareVER\":\"" + TERSE_SW_VERSION  + 
                 "\",\"timeStamp\":\""   + currentTime +  // Updated in main loop from RTC
                 "\",\"eventType\":\""   + eventType + 
                 "\""; 
                   
  return jsonHeader;
}


//****************************************************************************************
void handleModeButtonPress(){
  
  if (digitalRead(CTR_RESET)== LOW) {
    displayMode++;
    if (displayMode>3) displayMode = 1;
    switch (displayMode) {
      case 1:
        displaySplashScreen("(Base Station)",SW_VERSION);
        break;
      case 2:
        displayIPScreen(WiFi.localIP().toString()); 
        break;
      case 3:
        //displayTextScreen("STATUS","    Listening\n\n       ...");
        displayCountersSummaryScreen(strTotal,getCounterSummary());
        break;     
    }
  }   
  
}


//****************************************************************************************
// Experimenting with using a circular buffer and multi-tasking to enqueue 
// messages to the server...
void messageManager(void *parameter){
  String activeMessage;
  String jsonPayload;

  debugUART.print("Message Manager Running on Core #: ");
  debugUART.println(xPortGetCoreID());
  
  for(;;){  

    //Serial.println("messageManager TICK");
    //**********************************************
    // Check if we need to send a boot message
    //**********************************************      
    if (bootMessageNeeded){

      jsonPayload = buildJSONHeader("boot");
      jsonPayload = jsonPayload + "}";

      //debugUART.println(jsonPayload);
      postJSON(jsonPayload, config);

      xSemaphoreTake(mutex_v, portMAX_DELAY); 
        bootMessageNeeded = false;
      xSemaphoreGive(mutex_v);
      
    }
    

    //**********************************************
    // Check if we need to send a heartbeat message
    //**********************************************      
    if (heartbeatMessageNeeded){

      jsonPayload = buildJSONHeader("heartbeat");
      jsonPayload = jsonPayload + "}";

      //debugUART.println(jsonPayload);
      postJSON(jsonPayload, config);

      xSemaphoreTake(mutex_v, portMAX_DELAY); 
        heartbeatMessageNeeded = false;
      xSemaphoreGive(mutex_v);
      
    }
    
    //********************************************
    // Check for Messages on the Queue and Process
    //********************************************
    if (loraMsgBuffer.size()>0){

      xSemaphoreTake(mutex_v, portMAX_DELAY); 
        activeMessage=loraMsgBuffer.shift();
      xSemaphoreGive(mutex_v);

      processLoRaMessage(activeMessage);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);   
  }   
}

void eventDisplayManager(void *parameter){
  int eventDisplayUpdateRate = 20;
  String oldStrDisplay;

  #if !(SHOW_DATA_STREAM)
    debugUART.print("Display Manager Running on Core #: ");
    debugUART.println(xPortGetCoreID());
  #endif 
  
  for(;;){  

    if (strDisplay != oldStrDisplay){
        oldStrDisplay = strDisplay;        
        //displayEventScreen(strDisplay);
        displayCountersSummaryScreen(strTotal,getCounterSummary());
      }       
  
    vTaskDelay(eventDisplayUpdateRate / portTICK_PERIOD_MS);
  }
}
    

String getCounterSummary(){
  String retVal  = "";
  retVal += "   Ctr.   Count\n";
  retVal += " ---------------\n";
  retVal += "    1      " + str1Count + "\n";
  retVal += "    2      " + str2Count + "\n";
  retVal += "    3      " + str3Count + "\n";
  retVal += "    4      " + str4Count + "\n";
  return retVal;
}

//****************************************************************************************
// Setup
//****************************************************************************************
void setup() {
  
  initPorts();                      // Set up serial ports and GPIO
  splash();                         // Title, copyright, etc.
  initJSONConfig(filename, config); // Load the program config parameters 

  // Check for an unconfigured base station or RESET button pressed at boot. 
  if ((config.ssid == "YOUR_SSID") || (digitalRead(CTR_RESET)== LOW)){
    
    // -- Enter Access Point Mode to configure.
    accessPointMode = true; 
    debugUART.println("*******************************");
    debugUART.println("Launching in Access Point Mode!");  
    debugUART.println("*******************************");
    debugUART.println("Setting AP (Access Point)…");
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("  AP IP address: ");
    Serial.println(IP);
    server.begin();
    displayAPScreen(ssid, WiFi.softAPIP().toString());  
   
  }else{
    
    // -- Running in Normal Mode.
    debugUART.println("    Device Name: " + config.deviceName);
    debugUART.println("    SSID: " + config.ssid);
    debugUART.println("    ServerURL: " + config.serverURL);
    
    setFullPowerMode(); //Run at full power and max speed with WiFi enabled by default
    delay(500);
    enableWiFi(config);
    
    myMACAddress = getMACAddress();
    debugUART.println("    MAC Address: " + myMACAddress);
      
    configureLoRa(config);
    
    initRTC();

    if (wifiConnected){ // Attempt to synch ESP32 clock with NTP Server...
      synchTimesToNTP();
      displayIPScreen(WiFi.localIP().toString());
      delay(5000);
    }

    bootMinute = getRTCMinute();
    heartbeatMinute = bootMinute;
    oldheartbeatMinute = heartbeatMinute; 
    currentTime = getRTCTime();


  //WiFi.mode(WIFI_AP);
  //WiFi.softAP("esp32");
  
  //Serial.print("local IP address: ");
  //Serial.println(WiFi.softAPIP());

       
    mutex_v = xSemaphoreCreateMutex();  //The mutex we will use to protect the jsonMsgBuffer
    
    // Create a task that will be executed in the messageManager() function, 
    //   with priority 0 and executed on core 0
    xTaskCreatePinnedToCore(
      messageManager,      /* Task function. */
      "Message Manager",   /* name of task. */
      10000,               /* Stack size of task */
      NULL,                /* parameter of the task */
      0,                   /* priority of the task */
      &messageManagerTask, /* Task handle to keep track of created task */
      0);                  /* pin task to core 0 */ 

    // Create a task that will be executed in the CountDisplayManager() function, 
    // with priority 0 and executed on core 0
    xTaskCreatePinnedToCore(
      eventDisplayManager, /* Task function. */
      "Display Manager",   /* name of task. */
      10000,               /* Stack size of task */
      NULL,                /* parameter of the task */
      0,                   /* priority of the task */
      &eventDisplayManagerTask, /* Task handle to keep track of created task */
      0);

    displayMode=3;
    //displayTextScreen("STATUS","    Listening\n\n       ...");
    displayCountersSummaryScreen(strTotal,getCounterSummary());
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);
  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname(String(String("Digame-STN-") + getMACAddress()).c_str());

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();


  server.begin(); // Start the web server

  debugUART.println();
  debugUART.println("RUNNING\n");
  
}


//****************************************************************************************
// Main Loop
//****************************************************************************************
void loop() {
  String loraMsg;

  // OTA kick
   ArduinoOTA.handle();
  
  // For base stations, we always have a web server available.
    WiFiClient client = server.available();   // Listen for incoming clients
    if (client){ 
      processWebClient("basestation", client, config); // Web interface to tweak params and save
      initJSONConfig(filename, config); // Load the program config parameters 
    }
  
  //**************************************************************************************
  //Access Point Operation
  //**************************************************************************************
  if (accessPointMode){
 
    //Nothing to do here... 
    
  //**************************************************************************************
  //Standard Operation  
  //**************************************************************************************
  } else {
    
    // Grab the current time
      currentTime = getRTCTime();
    
    // Check for display mode button being pressed and switch display
      handleModeButtonPress();
    
    // Issue a heartbeat message every hour.
      heartbeatMinute = getRTCMinute();
      if ((oldheartbeatMinute != bootMinute) && (heartbeatMinute == bootMinute)){ 
        xSemaphoreTake(mutex_v, portMAX_DELAY); 
          heartbeatMessageNeeded = true;
        xSemaphoreGive(mutex_v);
      }  
      oldheartbeatMinute = heartbeatMinute;


    // Handle what the LoRa module has to say. 
    // If it's a message from another module, add it to the queue so the manager  
    // function can handle it.
    //
    // Otherwise, just echo to the debugUART.
      if (LoRaUART.available()) {
        
        loraMsg = LoRaUART.readStringUntil('\n');
           
        //Messages received by the Module start with '+RCV'
        if (loraMsg.indexOf("+RCV")>=0){
          debugUART.println("LoRa Message Received: ");  
          debugUART.println(loraMsg);
          
          // Send an acknowlegement to the sender.
          // Messages are of the form: "+RCV=2,2,16,-64,36" -- where the first number 
          // after the "=" is the sender's address.
          
          // Start and end of the JSON payload in the msg.
           int idxstart = loraMsg.indexOf('=')+1;
           int idxstop = loraMsg.indexOf(',');
        
          // Grab the address of the sender
          String senderAddress = loraMsg.substring(idxstart,idxstop); 
        
          // Let the sender know we got the message.
          debugUART.println("Sending ACK. ");
          debugUART.println();
          sendReceiveReyax("AT+SEND=" + senderAddress + ",3,ACK"); 
  
          // Put the message we received on the queue to process
          xSemaphoreTake(mutex_v, portMAX_DELAY);      
            loraMsgBuffer.push(loraMsg);
          xSemaphoreGive(mutex_v);
      
        } else {
          debugUART.println(loraMsg);      
        }
             
      }
    
  }
  
}
