     /* HEIMDALL VCS - Vehicle Counting Systems 
 * A traffic counting system that uses LIDAR to track pedestrians and vehicles.
 * 
 * Copyright 2021, Digame Systems. All rights reserved.
 */

#include <digameDebug.h> 

#define debugUART Serial

// Pick one LoRa or WiFi as the data reporting link. These are mutually exclusive.
#define USE_LORA false    // Use LoRa as the reporting link
#define USE_WIFI true     // Use WiFi as the reporting link

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
                                           // (JSON format depends on link type: LoRa or WiFi)

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
int    currentSecond;      // Set in the main loop from the RTC

// Instead of limiting heartbeats to an hourly schedule, make more flexible, based on a variable
unsigned long lastHeartbeatMillis = 0; 

unsigned long bootMillis=0;



//****************************************************************************************
// Pretty(?) debug splash screen
void splash(){
  
  String compileDate = F(__DATE__);
  String compileTime = F(__TIME__);

  DEBUG_PRINTLN("***************************************************************");
  DEBUG_PRINTLN("              HEIMDALL Vehicle Counting System");
  DEBUG_PRINTLN("                 - VEHICLE COUNTING UNIT -");
  DEBUG_PRINTLN();
  DEBUG_PRINT(model);
  DEBUG_PRINTLN(model_description);
  DEBUG_PRINTLN("Version: " + SW_VERSION);
  DEBUG_PRINT("Main Loop Running on Core #: ");
  DEBUG_PRINTLN(xPortGetCoreID());
  DEBUG_PRINTLN("Copyright 2021, Digame Systems. All rights reserved.");
  DEBUG_PRINTLN();
  DEBUG_PRINT("Compiled on ");
  DEBUG_PRINT(compileDate);
  DEBUG_PRINT(" at ");
  DEBUG_PRINTLN(compileTime); 
  DEBUG_PRINT("Free Heap: ");
  DEBUG_PRINTLN(ESP.getFreeHeap());
  DEBUG_PRINTLN("***************************************************************");
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("HARDWARE INITIALIZATION");
  DEBUG_PRINTLN();

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
               "\",\"r\":\"" + "0" ;               // Retries 
     
  if (eventType =="v"){
    loraHeader = loraHeader + 
                 "\",\"da\":\"" + "t";             // Detection algorithm (Threshold)       
    loraHeader = loraHeader + 
                 "\",\"l\":\"" + lane + "\"";      // Lane number for the vehicle event
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

  // DEBUG_PRINTLN(loraHeader);
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
// LoRa messages to the server all have a similar format. This builds the common header.
String buildJSONHeader(String eventType, double count, String lane = "1"){
  String retValue = "";
  
  #if USE_LORA
    retValue = buildLoRaJSONHeader(eventType, count, lane);
    //DEBUG_PRINTLN(retValue);
    return retValue;
  #endif

  #if USE_WIFI
    return buildWiFiJSONHeader(eventType, count, lane);
  #endif

}

/* Playing around with scheduling message delivery to minimize interference between LoRa counters. 
 *  Divide the minute up into equal portions for the number of counters
 *  we are dealing with. Each counter has his own window within the minute to transmit. 
 *  
 *  TODO: Think about having the base station tell us
 *  how many friends we have in the area and which counter we are... 
 *  
 *  counterNumber = 1 to 4
 *  numCounters   = 1 to 4
 */
bool inTransmitWindow(int counterNumber, int numCounters = 3){
  
  int thisSecond;
  thisSecond = currentSecond;  // Set in the main loop from the RTC

  // return true; // Uncomment to turn off time window check and allow counters to respond at any time.

   if (config.showDataStream == "false"){
     DEBUG_PRINT("This Second: ");
     DEBUG_PRINTLN(thisSecond);
   }

  int windowInterval = 60 / (numCounters);

  // counter 0: 0-14,      
  // counter 1:      15-29,      
  // counter 2:            30-44,      
  // counter 3:                  45-59

  if ( (thisSecond >=   (counterNumber-1) * windowInterval ) &&
       (thisSecond <  ( (counterNumber-1) * windowInterval + windowInterval ) ) 
     )
  {
    return true;
  }else{ 
    return false;
  }

}

//****************************************************************************************
// A task that runs on Core0 using a circular buffer to enqueue messages to the server...
// Current version keeps retrying forever if an ACK isn't received. TODO: Fix.
void messageManager(void *parameter){
  
  bool messageACKed=true; 
  
  DEBUG_PRINT("Message Manager Running on Core #: ");
  DEBUG_PRINTLN(xPortGetCoreID());
  DEBUG_PRINTLN();
  
  for(;;){  

    //*******************************
    // Process a message on the queue
    //*******************************

    if ( (msgBuffer.size() > 0) && (inTransmitWindow(config.counterID.toInt(), config.counterPopulation.toInt())) ){ 
    
      if (config.showDataStream == "false"){
        DEBUG_PRINT("Buffer Size: ");
        DEBUG_PRINTLN(msgBuffer.size());
      }
      
      String activeMessage = String(msgBuffer.first()->c_str()); // Read from the buffer without removing the data from it.
      
      // Send the data to the LoRa-WiFi base station that re-formats and routes it to the 
      // ParkData server.
      #if USE_LORA     
        // TODO: Come up with a reasonable re-try scheme and deal with failures more 
        // gracefully. This function simply yells out the message over the LoRa link and
        // returns true if it gets an ACK from the basestation. It will retry forever...
        // Probably not what we want.   
        //
        // Later: -- Rethinking that opinion. -- Daniel had his base station go down and 
        // when it came back up, the counter uploaded all the data it had taken since it 
        // went down. We lost _nothing_.
        messageACKed = sendReceiveLoRa(activeMessage);                                             
      #endif 
      
      // Send the data directly to the ParkData server via http(s) POST
      #if USE_WIFI
        messageACKed = postJSON(activeMessage, config);
      #endif   
      
      if (messageACKed) {
        
        // Message sent and received. Take it off of the queue. 
        xSemaphoreTake(mutex_v, portMAX_DELAY); 
          String  * entry = msgBuffer.shift();
          delete entry;
        xSemaphoreGive(mutex_v);
       
        if (config.showDataStream=="false"){
          DEBUG_PRINTLN("Success!");
          DEBUG_PRINTLN();
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
      
      } else {
        
        // Introduce a variable retry delay if we are in the transmit window.
        if (inTransmitWindow(config.counterID.toInt(), config.counterPopulation.toInt())) {
          
          if (config.showDataStream == "false"){
            DEBUG_PRINTLN("******* Timeout Waiting for ACK **********");
            DEBUG_PRINT("Retrying...");
          }      
           
          /* Playing with scheduling each counter in its own time window. 
           *  Commenting out the random backoff in that case. 
           *  
           *  Add a random delay before retrying... 4 time slots for retries.
          long backOffTime = 100 / portTICK_PERIOD_MS + (2000 * random(0,4)) / portTICK_PERIOD_MS;
          DEBUG_PRINTLN(backOffTime);
          vTaskDelay(backOffTime);
          */
              
          vTaskDelay(100 / portTICK_PERIOD_MS);    
          
        } else {
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        
      }
    } else {
      #if USE_WIFI
        if (wifiConnected){
          if ((millis() - msLastPostTime)>60000){
            DEBUG_PRINTLN("WiFi Stale...");
            disableWiFi();  
          }  
        }
      #endif
      vTaskDelay(100 / portTICK_PERIOD_MS);
      
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
    DEBUG_PRINT("Display Manager Running on Core #: ");
    DEBUG_PRINTLN(xPortGetCoreID());
    DEBUG_PRINTLN();
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

    //DEBUG_PRINT("Free Heap: ");
    //DEBUG_PRINTLN(ESP.getFreeHeap());

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
       DEBUG_PRINT("Loop: RESET button pressed. Count: ");
       DEBUG_PRINTLN(count); 
    }
  }  
}

//****************************************************************************************   
// Configure the device as an access point. TODO: move to DigameNetwork.h
void setupAPMode(const char* ssid){
  
  DEBUG_PRINTLN("  Stand-Alone Mode. Setting AP (Access Point)…");  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid);
  IPAddress IP = WiFi.softAPIP();
  DEBUG_PRINT("    AP IP address: ");
  DEBUG_PRINTLN(IP);   
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
  // Recently added a check to see if the unit was booted with something in front of the 
  // sensor. If so, come up in AP mode. -- This way folks don't have to press the button at 
  // boot.
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
    debugUART.println("  Configuring LoRa...");
    if (configureLoRa(config)){  
      hwStatus+="   LoRa : OK\n\n";
    }else{
      hwStatus+="   LoRa : ERROR!\n\n";  
    }
    
  #endif

  
  //**************************************************************************************   
  // Data reporting over the WiFi link
  #if USE_WIFI

    useOTA = true;   
    usingWiFi = true;

    //Experiment: Try slowing down the cpu to 80MHz and keeping WiFi on... Saves c.a. 20mA
    setMediumPowerMode(); // See: DigamePowerMangement.h
    
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
  DEBUG_PRINT("  USE OTA: ");
  DEBUG_PRINTLN(useOTA);
  if (useOTA){
    //initOTA();    
  }
    

  //**************************************************************************************   
  // Check that the RTC is present
  if (initRTC()) { 
    hwStatus+="   RTC  : OK\n";
  }else{
    hwStatus+="   RTC  : ERROR!\n";
  }

   
  //**************************************************************************************   
  // Synchronize the RTC to an NTP server, if available
  #if USE_WIFI
    if (wifiConnected){ // Attempt to synch ESP32 clock with NTP Server...
      synchTimesToNTP();
    }
  #endif

  DEBUG_PRINTLN();
  
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
  lastHeartbeatMillis = millis();

// ********* TESTING....************
 //config.showDataStream = "true"; 
  
  currentTime = getRTCTime(); 
  bootMinute = getRTCMinute();
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("RUNNING!");  
  DEBUG_PRINTLN(); 
  
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
  
  unsigned long T1,T2;
   
  T1 = millis(); // Time at the start of the loop.
    
  if (resetFlag){
    DEBUG_PRINTLN("Reset flag has been flipped. Rebooting the processor.");
    delay(1000);  
    ESP.restart();
  }

  //**************************************************************************************   
  // Grab the current time
  currentTime   = getRTCTime();
  currentSecond = getRTCSecond();
    
  //**************************************************************************************
  // Event management and notification
  //**************************************************************************************       
  
  
  //**************************************************************************************   
  // Test if vehicle event has occured 
  // TODO: Consider putting this in a separate RTOS Task. 
    
    vehicleMessageNeeded = processLIDARSignal3(config); //
   
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
  // Issue a heartbeat message 
    /*if ( ( (millis()-lastHeartbeatMillis)/1000 ) >= config.heartbeatInterval.toInt() ){
      heartbeatMessageNeeded = true;
    }*/

    unsigned long deltaT = (millis() - lastHeartbeatMillis);
    unsigned long slippedMilliSeconds=0; 
    if ( (deltaT) >= config.heartbeatInterval.toInt() * 1000 ){
       if (config.showDataStream == "false"){
         DEBUG_PRINT("millis: ");
         DEBUG_PRINTLN(deltaT);
       }
      slippedMilliSeconds = deltaT - config.heartbeatInterval.toInt() *1000; // Since this Task is on a 100 msec schedule, we'll always be a little late...
      if (config.showDataStream == "false"){     
        DEBUG_PRINT("slipped ms: ");
        DEBUG_PRINTLN(slippedMilliSeconds);
      }
      heartbeatMessageNeeded = true;
    }
    
    if (heartbeatMessageNeeded){
      msgPayload = buildJSONHeader("hb",count);
      msgPayload = msgPayload + "}";
      pushMessage(msgPayload);
      if (config.logHeartBeatEvents=="checked"){
        appendTextFile("/eventlog.txt",msgPayload);
      }      
      heartbeatMessageNeeded = false; 
      lastHeartbeatMillis = millis()- slippedMilliSeconds;
      
    }
   
  //**************************************************************************************   
  // Issue a vehicle event message
    if (vehicleMessageNeeded>0){ 
      if (lidarBuffer.size()==lidarSamples){ // Fill up the buffer before processing so 
                                             // we don't get false events at startup.      
        count++;     
        config.lidarZone1Count = String(count); // Update this so entities making use of config have access to the current count.
                                                // e.g., digameWebServer.h
                                                
        if (config.showDataStream == "false"){
          DEBUG_PRINT("Vehicle event! Counts: ");
          DEBUG_PRINTLN(count);
        }
        
        if (vehicleMessageNeeded==1){
          if (config.showDataStream == "false"){
            DEBUG_PRINTLN("LANE 1 Event!");
          }
          
          msgPayload = buildJSONHeader("v",count,"1");
        }

        if (vehicleMessageNeeded==2){
          if (config.showDataStream == "false"){
            DEBUG_PRINTLN("LANE 2 Event!");
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

  delay(5);

  
  T2= millis();
  //DEBUG_PRINTLN(T2-T1); 
   
}
