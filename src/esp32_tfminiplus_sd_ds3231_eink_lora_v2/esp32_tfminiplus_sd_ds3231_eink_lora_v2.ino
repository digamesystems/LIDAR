 /* HEIMDALL VCS - Vehicle Counting Systems 
 * A traffic counting system that uses LIDAR to track pedestrians and vehicles.
 * 
 * Copyright 2021, Digame Systems. All rights reserved.
 */
#define debugUART Serial

// Pick one LoRa or WiFi as the data reporting link. These are mutually exclusive.
#define USE_LORA false    // Use LoRa as the reporting link
#define USE_WIFI true   // Use WiFi as the reporting link

#if USE_LORA
  String model = "DS-VC-LIDAR-LORA-1";
  String model_description= "(LIDAR Traffic Counter with LoRa Back Haul)";  
#endif

#if USE_WIFI
  String model = "DS-VC-LIDAR-WIFI-1";
  String model_description= "(LIDAR Traffic Counter with WiFi Back Haul)";  
  #define APPEND_RAW_DATA_WIFI true // In USE_WIFI mode, add 100 points of raw LIDAR data to the wifi 
                                    // JSON msg for analysis at the server.
#endif

                                 
#include <digameVersion.h>    // Global SW version constants
#include <digameDebug.h> 
#include <digameJSONConfig.h> // Program parameters from config file on SD card. Set up the config struct that is used all over the place.
#include <digameTime.h>       // Time Functions - RTC, NTP Synch etc
#include <digameNetwork.h>    // Network Functions - Login, MAC addr
//#include <digameOTA.h>        // Over the Air Updates -- Playing with replacing with ElegantOTA library in digameCounterWebServer.h

unsigned long count = 0;          // The number of vehicle events recorded

#include <digameCounterWebServer.h>  // Web page to tweak parameters
#include <digamePowerMgt.h>   // Power management modes 
#include <digameDisplay.h>    // eInk Display Functions
#include <digameLIDAR.h>      // Functions for working with the TFMini series LIDAR sensors

#if USE_LORA
  #include <digameLoRa.h>     // Functions for working with Reyax LoRa module
#endif 

#include <CircularBuffer.h>   // Adafruit library for handling circular buffers of data. 

const int samples = 200;

CircularBuffer<String *, samples> msgBuffer; // A buffer containing pointers JSON messages to be 
                                             // sent to the LoRa basestation or server.

String msgPayload;                         // The message being sent to the base station. 
                                           // (JSON format depends on Link LoRa or WiFi)

// Access point mode variables
bool accessPointMode  = false;             // Flag used at boot 
bool usingWiFi = false;                    // True if USE_WIFI or AP mode is enabled

// Multi-Tasking
SemaphoreHandle_t mutex_v;        // Mutex used to protect variables across RTOS tasks. 
TaskHandle_t messageManagerTask;  // A task for handling data reporting
TaskHandle_t displayManagerTask;  // A task for updating the EInk display

// Messaging flags
bool   jsonPostNeeded         = false;
bool   bootMessageNeeded      = true; // Send boot and heartbeat messages at startup
bool   heartbeatMessageNeeded = true;
int    vehicleMessageNeeded   = 0;    // 0=false, 1=lane 1, 2=lane 2 messages

// Over the Air Updates work when we have WiFi or are in Access Point Mode...
bool useOTA = true;

// DIO
int    LED_DIAG  = 12;     // Indicator LED
int    CTR_RESET = 32;     // Counter Reset Input

// Bookeeping
String myMACAddress;       // Written at boot
int    bootMinute;         // The minute (0-59) within the hour we woke up at boot. 
int    heartbeatMinute;    // Issue Heartbeat message once an hour. Holds the current Minute.
int    oldheartbeatMinute; // Value the last time we looked.
String currentTime;        // Set in the main loop from the RTC

unsigned long bootMillis=0;


//****************************************************************************************
// Pretty(?) debug splash screen
void splash(){
  
  String compileDate = F(__DATE__);
  String compileTime = F(__TIME__);

  debugUART.println("*****************************************************");
  debugUART.println("         HEIMDALL Vehicle Counting System");
  debugUART.println("            - VEHICLE COUNTING UNIT -");
  debugUART.println();
  debugUART.print(model);
  debugUART.println(model_description);
  debugUART.println("Version: " + SW_VERSION);
  debugUART.print("Main Loop Running on Core #: ");
  debugUART.println(xPortGetCoreID());
  debugUART.println("Copyright 2021, Digame Systems. All rights reserved.");
  debugUART.println();
  debugUART.print("Compiled on ");
  debugUART.print(compileDate);
  debugUART.print(" at ");
  debugUART.println(compileTime); 
  debugUART.print("Free Heap: ");
  debugUART.println(ESP.getFreeHeap());
  debugUART.println("*****************************************************");
  debugUART.println();
  debugUART.println("HARDWARE INITIALIZATION");
  debugUART.println();
  
      
}


//****************************************************************************************
// Prepare the GPIOs and debugUART serial port. Bring up the Wire interface for SPI bus.
void initPorts(){
  
    // Ready the LED and RESET pins
    pinMode(LED_DIAG, OUTPUT);
    pinMode(CTR_RESET,INPUT_PULLUP);      
    debugUART.begin(115200);   // Intialize terminal serial port
    delay(1000);               // Give port time to initalize
    Wire.begin(); 
    
}


//****************************************************************************************
void initUI(){
    splash();   
}


#if USE_LORA
//****************************************************************************************
// LoRa can't handle big payloads. We use a terse JSON message in this case.
String buildLoRaJSONHeader(String eventType, double count, String lane="1"){
  String loraHeader;
  String strCount;

  strCount= String(count,0);
  strCount.trim();
  
  if (rtcPresent()){
    loraHeader = "{\"ts\":\"" + getRTCTime(); // Timestamp
  } else {
    loraHeader = "{\"ts\":\"" + getESPTime(); 
  }  
  
  loraHeader = loraHeader +
               "\",\"v\":\""  + TERSE_SW_VERSION + // Firmware version
               "\",\"et\":\"" + eventType +        // Event type: boot, heartbeat, vehicle
               "\",\"c\":\""  + strCount +         // Total counts registered
               "\",\"t\":\""  + String(getRTCTemperature(),1) + // Temperature in C
               "\",\"r\":\"" + "0" ;             // Retries 
     
  if (eventType =="v"){
    loraHeader = loraHeader + 
                 "\",\"da\":\"" + "t";           // Detection algorithm (Threshold)       
    loraHeader = loraHeader + 
                 "\",\"l\":\"" + lane + "\"";           // Lane number for the vehicle event
  }             
               
  if ((eventType =="b")||(eventType=="hb")){ //In the boot/heartbeat messages, send the current settings.
    loraHeader = loraHeader +
                 "\",\"s\":{" +
                    "\"ui\":\"" + config.lidarUpdateInterval  + "\"" +
                   ",\"sf\":\"" + config.lidarSmoothingFactor + "\"" +
                   ",\"rt\":\"" + config.lidarResidenceTime   + "\"" +
                   ",\"1m\":\"" + config.lidarZone1Min        + "\"" +
                   ",\"1x\":\"" + config.lidarZone1Max        + "\"" +
                   ",\"2m\":\"" + config.lidarZone2Min        + "\"" +
                   ",\"2x\":\"" + config.lidarZone2Max        + "\"" +
                 "}"; 
  }

  // debugUART.println(loraHeader);
  return loraHeader;
  
}
#endif 


#if USE_WIFI
//****************************************************************************************
// WiFi can handle a more human-readable JSON data payload. 
String buildWiFiJSONHeader(String eventType, double count, String lane ="1"){
  String jsonHeader;
  String strCount;

  strCount= String(count,0);
  strCount.trim();

  if (eventType =="b") eventType  = "Boot";
  if (eventType =="hb") eventType = "Heartbeat";
  if (eventType =="v") eventType  = "Vehicle";
  
  jsonHeader = "{\"deviceName\":\""      + config.deviceName + 
                 "\",\"deviceMAC\":\""   + myMACAddress +      // Read at boot
                 "\",\"firmwareVer\":\"" + TERSE_SW_VERSION  + 
                 "\",\"timeStamp\":\""   + currentTime +       // Updated in main loop from RTC
                 "\",\"eventType\":\""   + eventType +
                 "\",\"count\":\""       + strCount +          // Total counts registered
                 "\",\"temp\":\""        + String(getRTCTemperature(),1); // Temperature in C
  
  if (eventType =="Vehicle"){
    jsonHeader = jsonHeader + 
                 "\",\"detAlgorithm\":\"" + "Threshold";// Detection algorithm (Threshold)
    jsonHeader = jsonHeader + 
                 "\",\"lane\":\"" + lane + "\"";        // Lane in which the vehicle was seen   
       
  }            

  if ((eventType =="Boot")||(eventType=="Heartbeat")){ //In the boot/heartbeat messages, send the current settings.
    jsonHeader = jsonHeader +
                 "\",\"settings\":{" +
                    "\"ui\":\"" + config.lidarUpdateInterval  + "\"" +
                   ",\"sf\":\"" + config.lidarSmoothingFactor + "\"" +
                   ",\"rt\":\"" + config.lidarResidenceTime   + "\"" +
                   ",\"1m\":\"" + config.lidarZone1Min        + "\"" +
                   ",\"1x\":\"" + config.lidarZone1Max        + "\"" +
                   ",\"2m\":\"" + config.lidarZone2Min        + "\"" +
                   ",\"2x\":\"" + config.lidarZone2Max        + "\"" +
                 "}"; 
  } 

  //  jsonHeader = jsonHeader + "\""; 
                                   
  return jsonHeader;  
}
#endif


//****************************************************************************************
// LoRa messages to the server all have a similar format. 
String buildJSONHeader(String eventType, double count, String lane = "1"){
  String retValue = "";
  
  #if USE_LORA
    retValue = buildLoRaJSONHeader(eventType, count, lane);
    //debugUART.println(retValue);
    return retValue;
  #endif

  #if USE_WIFI
    return buildWiFiJSONHeader(eventType, count, lane);
  #endif

}


//****************************************************************************************
// A task that runs on Core0 using a circular buffer to enqueue messages to the server...
// Current version keeps retrying forever if an ACK isn't received. TODO: Fix.
void messageManager(void *parameter){
  
  bool messageACKed=true; 
  
  debugUART.print("Message Manager Running on Core #: ");
  debugUART.println(xPortGetCoreID());
  debugUART.println();
  
  for(;;){  

    //*******************************
    // Process a message on the queue
    //*******************************
    if (msgBuffer.size()>0){ 

      if (config.showDataStream == "false"){
        debugUART.print("Buffer Size: ");
        debugUART.println(msgBuffer.size());
      }
      
      String activeMessage = String(msgBuffer.first()->c_str()); // Read from the buffer without removing the data from it.
          
      // Send the data to the LoRa-WiFi base station that re-formats and routes it to the 
      // ParkData server.
      #if USE_LORA     
        // TODO: Come up with a reasonable re-try scheme and deal with failures more 
        // gracefully. This function simply yells out the message over the LoRa link and
        // returns true if it gets an ACK from the basestation. It will retry forever...
        // Probably not what we want.   
         messageACKed = sendReceiveLoRa(activeMessage);                                             
      #endif 

      // Send the data directly to the ParkData server via http(s) POST
      #if USE_WIFI
        messageACKed = postJSON(activeMessage, config);
        //messageACKed = true;
      #endif   

      if (messageACKed) {
        // Message sent and received. Take it off of the queue. 
        xSemaphoreTake(mutex_v, portMAX_DELAY); 
            String  * entry = msgBuffer.shift();
            delete entry;
        xSemaphoreGive(mutex_v);
        
        
        if (config.showDataStream=="false"){
          debugUART.println("Success!");
          debugUART.println();
        }
        
      } 
      
    }

    //**************************************************************************************   
    // Let the Over the Air Programming process have some CPU
    if (useOTA){
      //ArduinoOTA.handle();
    }
    
    if (messageACKed){
      vTaskDelay(100 / portTICK_PERIOD_MS);
    } else {
      if (config.showDataStream == "false"){
        debugUART.println("*******MESSAGE COLLISION**********");
        debugUART.print("Delaying: ");
      }
      // Add a random delay before retrying... 4 time slots for retries.
      long backOffTime = 100 / portTICK_PERIOD_MS + (2000 * random(0,4)) / portTICK_PERIOD_MS;
      debugUART.println(backOffTime);
      vTaskDelay(backOffTime); 
    }   
  
  }   
  
}


//****************************************************************************************
// A task that runs on Core0 to update the display when the count changes. 
void countDisplayManager(void *parameter){
  int countDisplayUpdateRate = 200;
  unsigned int myCount;
  static unsigned int oldCount=0;
  
  if (config.showDataStream == "false"){
    debugUART.print("Display Manager Running on Core #: ");
    debugUART.println(xPortGetCoreID());
    debugUART.println();
  }
  
  for(;;){  
      
    myCount = count;

    if (myCount != oldCount){
      // Total refresh every 100 counts. (Or when we zero out the counter.)
      //if (displayType=="SSD1608"){
        if ((myCount%100==0)) {
          initDisplay();
          displayCountScreen(myCount);
        }       
      //}
      showValue(myCount);  
      oldCount = myCount;
    }

    //debugUART.print("Free Heap: ");
    //debugUART.println(ESP.getFreeHeap());

    upTimeMillis = millis() - bootMillis; 
    vTaskDelay(countDisplayUpdateRate / portTICK_PERIOD_MS);

  }
    
}

//***************************************************************************************   
// Check for display mode button being pressed and reset the vehicle count
void handleModeButtonPress(){
  // Check for RESET button being pressed. If it has been, reset the counter to zero.
  if (digitalRead(CTR_RESET)== LOW) { 
    count = 0; 
    clearLIDARDistanceHistogram();
    if (config.showDataStream == "false"){
       debugUART.print("Loop: RESET button pressed. Count: ");
       debugUART.println(count); 
    }
  }  
}

//****************************************************************************************   
// Configure the device as an access point
void setupAPMode(const char* ssid){
  
  debugUART.println("  Stand-Alone Mode. Setting AP (Access Point)…");  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid);
  IPAddress IP = WiFi.softAPIP();
  debugUART.print("    AP IP address: ");
  debugUART.println(IP);   
  displayAPScreen(ssid, WiFi.softAPIP().toString());
  delay(3000);  
}

//****************************************************************************************
// Device initialization                                   
//****************************************************************************************
void setup(){
  
  String hwStatus = "";             // String to hold the results of self-test
  
  initPorts();                      // Set up UARTs and GPIOs
  initUI();                         // Splash screens

  String foo = "Digame-CTR-" + getShortMACAddress();
  const char* ssid = foo.c_str();

  //**************************************************************************************   
  // Setup SD card and load default values.
  if (initJSONConfig(filename, config))
  {
    hwStatus+="   SD   : OK\n\n";     
  }else{
    hwStatus+="   SD   : ERROR!\n\n"; 
  };
  
  //**************************************************************************************   
  // Show the splash screen
  initDisplay();
  displaySplashScreen("(LIDAR Counter)",SW_VERSION); 

  //**************************************************************************************   
  // Turn on the LIDAR Sensor
  if (initLIDAR(true)) { 
    hwStatus+="   LIDAR: OK\n\n";
  } else {
    hwStatus+="   LIDAR: ERROR!\n\n";
  }

 
  //**************************************************************************************   
  // If the unit is unconfigured or is booted with the RESET button held down, enter AP mode.
  if ((config.deviceName == "YOUR_DEVICE_NAME")||(digitalRead(CTR_RESET)== LOW) || (initLIDARDist <20) ) {
    accessPointMode = true;
    usingWiFi = true;
  } else {
    accessPointMode = false;  
  }

  //**************************************************************************************   
  // Data reporting over the LoRa link.
  #if USE_LORA  
    
    if (accessPointMode){
      useOTA = true;
      usingWiFi = true;
      setupAPMode(ssid); 
            
    }else{
      useOTA = false;
      usingWiFi = false;
      setLowPowerMode(); // Lower clock rate and turn off WiFi
    }

    // Initialize radio
    initLoRa();

    // Configure radio params 
    if (configureLoRa(config)){  
      hwStatus+="   LoRa : OK\n\n";
    }else{
      hwStatus+="   LoRa : ERROR!\n\n";  
    }
    
  #endif

  
  //**************************************************************************************   
  // Data reporting over the WiFi link
  #if USE_WIFI

    btStop();  // Turn off bluetooth to save power.
    adc_power_off(); // We're not using ADCs. Save a little power here, too.
    esp_bt_controller_disable();
  
    useOTA = true;   
    usingWiFi = true;
    
    myMACAddress = getMACAddress();        
    if (accessPointMode){
      setupAPMode(ssid);  
    }else{
      enableWiFi(config);
      displayIPScreen(String(WiFi.localIP().toString()));
      delay(3000);
      hwStatus += "   WiFi:  OK\n\n";
    }  
   
  #endif

  if (usingWiFi){
    initWebServer();
    // allow reuse on HTTPClient that does the POSTs (if server supports it)
    http.setReuse(true); // See digameNetwork.h for this guy.
   }


  //**************************************************************************************   
  debugUART.print("  USE OTA: ");
  debugUART.println(useOTA);
  if (useOTA){
    //initOTA();    
  }
    
  //**************************************************************************************   
  /*// Turn on the LIDAR Sensor
  if (initLIDAR(true)) { 
    hwStatus+="   LIDAR: OK\n\n";
  } else {
    hwStatus+="   LIDAR: ERROR!\n\n";
  }
  */

  //**************************************************************************************   
  // Check that the RTC is present
  if (initRTC()) { 
    hwStatus+="   RTC  : OK\n";
  }else{
    hwStatus+="   RTC  : ERROR!\n";
  }

  #if LOG_RAW_DATA_TO_SD 
    logFileName = getRTCTime(); 
    logFileName.replace(":","");
    
    logFileName = "/" + logFileName.substring(11) + ".txt";
  
    debugUART.println(logFileName);
    
    logFile = SD.open(logFileName, FILE_WRITE);  // See digameLIDAR.
 
    if (!logFile)
    {
      debugUART.println(F("    Failed to create log file!"));
    }
  #endif
   

  //**************************************************************************************   
  // Synchronize the RTC to an NTP server, if available
  #if USE_WIFI
    if (wifiConnected){ // Attempt to synch ESP32 clock with NTP Server...
      synchTimesToNTP();
    }
  #endif

  debugUART.println();
  
  //**************************************************************************************   
  // Show the results of the hardware configuration 
  displayStatusScreen(hwStatus); 
  delay(5000);


  //**************************************************************************************   
  // Set up the display to show vehicle events
  count=0;
  displayCountScreen(count);
  showValue(count);
  delay(1000);

  //displayBarcodeScreen();


  //**************************************************************************************   
  // Set up two tasks that run on CPU0 -- One to handle updating the display and one to 
  // handle reporting data
  mutex_v = xSemaphoreCreateMutex(); // The mutex we will use to protect variables 
                                     // across tasks

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
    countDisplayManager, /* Task function. */
    "Display Manager",   /* name of task. */
    10000,               /* Stack size of task */
    NULL,                /* parameter of the task */
    0,                   /* priority of the task */
    &displayManagerTask, /* Task handle to keep track of created task */
    0);

      
  
  //**************************************************************************************   
  // GO!
  bootMillis = millis();
  upTimeMillis = millis() - bootMillis;
  
  currentTime = getRTCTime(); 
  bootMinute = getRTCMinute();
  debugUART.println();
  debugUART.println("RUNNING!");  
  debugUART.println(); 
  
}


//**************************************************************************************   
  // Add a message to the queue for transmission.
void pushMessage(String message){
  xSemaphoreTake(mutex_v, portMAX_DELAY);  
    #if USE_LORA
      String * msgPtr = new String(message);
      msgBuffer.push(msgPtr);
    #endif

    #if USE_WIFI
      if (accessPointMode == false){
        String * msgPtr = new String(message);
        msgBuffer.push(msgPtr);
      }
    #endif
  xSemaphoreGive(mutex_v);  
}

//****************************************************************************************
// Main Loop
//****************************************************************************************
void loop(){ 

  if (resetFlag){
    debugUART.println("Reset flag has been flipped. Rebooting the processor.");
    delay(1000);  
    ESP.restart();
  }

  //**************************************************************************************   
  // Grab the current time
  currentTime = getRTCTime();
    
  //**************************************************************************************
  // Event management and notification
  //**************************************************************************************       
  
  
  //**************************************************************************************   
  // Test if vehicle event has occured 
  // TODO: Consider putting this in a separate RTOS Task. 
    vehicleMessageNeeded = processLIDARSignal2(config); // Threshold Detection Algorithm

  //**************************************************************************************   
  // Check for display mode button being pressed and switch display
    handleModeButtonPress();

  //**************************************************************************************   
  // Boot messages are sent at startup.
    if (bootMessageNeeded){
      msgPayload = buildJSONHeader("b",count);
      msgPayload = msgPayload + "}"; 
      pushMessage(msgPayload);
      if (config.logBootEvents=="checked"){
        appendTextFile("/eventlog.txt",msgPayload);
      }
      bootMessageNeeded = false;
    }

  //**************************************************************************************   
  // Issue a heartbeat message every hour.
    heartbeatMinute = getRTCMinute();
    if ((oldheartbeatMinute != bootMinute) && (heartbeatMinute == bootMinute)){heartbeatMessageNeeded = true;}  
    if (heartbeatMessageNeeded){
      msgPayload = buildJSONHeader("hb",count);
      msgPayload = msgPayload + "}";
      pushMessage(msgPayload);
      if (config.logHeartBeatEvents=="checked"){
        appendTextFile("/eventlog.txt",msgPayload);
      }
        
      
      heartbeatMessageNeeded = false; 
    }
    oldheartbeatMinute = heartbeatMinute;

  //**************************************************************************************   
  // Issue a vehicle event message
    if (vehicleMessageNeeded>0){ 
      if (lidarBuffer.size()==lidarSamples){ // Fill up the buffer before processing so 
                                             // we don't get false events at startup.      
        count++;     
        config.lidarZone1Count = String(count); // Update this so entities making use of config have access to the current count.
                                                // e.g., digameWebServer.h
                                                
        if (config.showDataStream == "false"){
          debugUART.print("Vehicle event! Counts: ");
          debugUART.println(count);
        }
        
        if (vehicleMessageNeeded==1){
          if (config.showDataStream == "false"){
            debugUART.println("LANE 1 Event!");
          }
          
          msgPayload = buildJSONHeader("v",count,"1");
        }

        if (vehicleMessageNeeded==2){
          if (config.showDataStream == "false"){
            debugUART.println("LANE 2 Event!");
          }
          msgPayload = buildJSONHeader("v",count,"2");
        }
        
         
        #if (USE_WIFI) && (APPEND_RAW_DATA_WIFI)
          // Vehicle passing event messages may include raw data from the sensor.
          // If so, tack the data buffer on the JSON message.
          msgPayload = msgPayload + ",\"rawSignal\":[";
          using index_t = decltype(lidarHistoryBuffer)::index_t;
          for (index_t i = 0; i < lidarHistoryBuffer.size(); i++) {
            msgPayload = msgPayload + lidarHistoryBuffer[i]; 
            if (i<lidarHistoryBuffer.size()-1){ 
              msgPayload = msgPayload + ",";
            }
          }
          msgPayload = msgPayload + "]";
        #endif
        
        msgPayload = msgPayload + "}";  
         
        pushMessage(msgPayload);
        if (config.logVehicleEvents=="checked"){
          appendTextFile("/eventlog.txt",msgPayload);
        }
        
      }
      vehicleMessageNeeded = 0; 
    }
}
