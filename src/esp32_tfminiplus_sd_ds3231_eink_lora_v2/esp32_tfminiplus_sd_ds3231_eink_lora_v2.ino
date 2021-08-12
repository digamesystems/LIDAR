/* HEIMDALL VCS - Vehicle Counting Systems 
 * A traffic counting system that uses LIDAR to track pedestrians and vehicles.
 * 
 * Copyright 2021, Digame Systems. All rights reserved.
 */

// Pick one LoRa or WiFi. These are mutually exclusive.
#define USE_LORA false         // Use LoRa as the reporting link
#define USE_WIFI true        // Use WiFi as the reporting link

#define APPEND_RAW_DATA_WIFI true   // In USE_WIFI mode, add 100 points of raw LIDAR data to the wifi 
                               // JSON msg for analysis.

#define STAND_ALONE_LORA false // In USE_LORA mode, this flag enables the AP web server and 
                               // disables the ACK requirement on LoRa messages, allowing you 
                               // to run without a base station and configure parameters through
                               // the web page. 

#define SHOW_DATA_STREAM false  // A debugging flag to show raw LIDAR values on the 
                                // serial monitor. -- This mode disables other output to the 
                                // serial monitor after bootup.

const String SW_VERSION       = "0.9.5";
const String TERSE_SW_VERSION = "095";  

#define debugUART Serial      // Alias for easier reading instead of "Serial, Serial1, Serial2, etc..."

#include <CircularBuffer.h>   // Adafruit library. Pretty small!

#include <digameJSONConfig.h> // Program parameters from config file on SD card
#include <digameTime.h>       // Time Functions - RTC, NTP Synch etc
#include <digameNetwork.h>    // Network Functions - Login, MAC addr
#include <digameWebServer.h>  // Web page to tweak parameters
#include <digamePowerMgt.h>   // Power management modes 
#include <digameDisplay.h>    // eInk Display Functions
#include <digameLIDAR.h>      // Functions for working with the TFMini series LIDAR sensors

#if USE_LORA
  #include <digameLoRa.h>     // Functions for working with Reyax LoRa module
#endif 

const int samples = 200;

CircularBuffer<String, samples> msgBuffer; // A buffer containing JSON messages to be 
                                           // sent to the LoRa basestation.

String msgPayload;                         // The message being sent to the base station. 
                                           // (JSON format depends on Link LoRa or WiFi)

// Access point mode variables
const char *ssid      = "Digame-CTR-AP";   // The name of our AP network
bool accessPointMode  = false;             // Flag used at boot 

Config config; // A structure to hold program configuration parameters. 
               // See: digameJSONConfig.h (Used all over the place)


// Multi-Tasking
SemaphoreHandle_t mutex_v; // Mutex used to protect variables across RTOS tasks. 
TaskHandle_t messageManagerTask;
TaskHandle_t displayManagerTask;

// Messaging flags
bool   jsonPostNeeded         = false;
bool   bootMessageNeeded      = true;
bool   heartbeatMessageNeeded = false;
bool   vehicleMessageNeeded   = false;

// DIO
int    LED_DIAG  = 12;     // Indicator LED
int    CTR_RESET = 32;     // Counter Reset Input

// Bookeeping
String myMACAddress;       // Written at boot
int    bootMinute;         // The minute (0-59) within the hour we woke up at boot. 
int    heartbeatMinute;    // Issue Heartbeat message once an hour. Holds the current Minute.
int    oldheartbeatMinute; // Value the last time we looked.
unsigned long count = 0;   // The number of vehicle events recorded
String currentTime;        // Set in the main loop from the RTC


//****************************************************************************************
void splash(){
  
  String compileDate = F(__DATE__);
  String compileTime = F(__TIME__);

  debugUART.println("*****************************************************");
  debugUART.println("         HEIMDALL Vehicle Counting System");
  debugUART.println("            - VEHICLE COUNTING UNIT -");
  debugUART.println();
  debugUART.println("Model: DS-VC-LIDAR-LOR-1 (LIDAR + LoRa Backhaul)");
  debugUART.println("Version: " + SW_VERSION);
  debugUART.print("Main Loop Running on Core #: ");
  debugUART.println(xPortGetCoreID());
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
String buildLoRaJSONHeader(String eventType, double count){
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
                 "\",\"da\":\"" + "t" + "\"";           // Detection algorithm (Threshold)       
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

  debugUART.println(loraHeader);
  return loraHeader;
  
}
#endif 


#if USE_WIFI
//****************************************************************************************
String buildWiFiJSONHeader(String eventType, double count){
  String jsonHeader;
  String strCount;

  strCount= String(count,0);
  strCount.trim();

  if (eventType =="b") eventType  = "boot";
  if (eventType =="hb") eventType = "heartbeat";
  if (eventType =="v") eventType  = "vehicle";
  

  jsonHeader = "{\"deviceName\":\""      + config.deviceName + 
                 "\",\"deviceMAC\":\""   + myMACAddress +      // Read at boot
                 "\",\"firmwareVer\":\"" + TERSE_SW_VERSION  + 
                 "\",\"timeStamp\":\""   + currentTime +       // Updated in main loop from RTC
                 "\",\"eventType\":\""   + eventType +
                 "\",\"count\":\""       + strCount +          // Total counts registered
                 "\",\"temp\":\""        + String(getRTCTemperature(),1); // Temperature in C


  if (eventType =="vehicle"){
    jsonHeader = jsonHeader + 
                 "\",\"detAlgorithm\":\"" + "Threshold\"";       // Detection algorithm (Threshold)       
  }            

  if ((eventType =="boot")||(eventType=="heartbeat")){ //In the boot/heartbeat messages, send the current settings.
    jsonHeader = jsonHeader +
                 "\",\"settings\":{" +
                    "\"ui\":\"" + config.lidarUpdateInterval  + "\"" +
                   ",\"sf\":\"" + config.lidarSmoothingFactor   + "\"" +
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
String buildJSONHeader(String eventType, double count){

  #if USE_LORA
    return buildLoRaJSONHeader(eventType, count);
  #endif

  #if USE_WIFI
    return buildWiFiJSONHeader(eventType, count);
  #endif

}


//****************************************************************************************
// Experimenting with using a circular buffer to enqueue messages to the server...
void messageManager(void *parameter){
  
  String activeMessage;

  debugUART.print("Message Manager Running on Core #: ");
  debugUART.println(xPortGetCoreID());
  
  for(;;){  

    //****************************
    //LoRa message
    //****************************
    if (msgBuffer.size()>0){ 
          
      xSemaphoreTake(mutex_v, portMAX_DELAY); 
        activeMessage=msgBuffer.shift();
      xSemaphoreGive(mutex_v);

      //sendDataHome(activeMessage, config);

      // Send the data to the LoRa-WiFi base station
      #if USE_LORA     
        // TODO: This runs forever if no base station is available.
        // Is this the right way to go? Consider adding a timeout.
        // If we do that, do we save the data to the SD card?
        // Come up with a scheme to send the buffered data when a
        // base station becomes available.      
        while (!sendReceiveLoRa(activeMessage)){};  

        //************************
        // Debugging for range checking! Undo later!!!
        //count++;                                            
      #endif 

      // Send the data to the ParkData server directly
      #if USE_WIFI
        while (!postJSON(activeMessage, config)){};       
      #endif   
      
      #if !(SHOW_DATA_STREAM)
        debugUART.println("Success!");
        debugUART.println();
      #endif
      
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS);   
  }   
  
}



void countDisplayManager(void *parameter){
  int countDisplayUpdateRate = 20;
  unsigned int myCount;
  static unsigned int oldCount=0;
  
  #if !(SHOW_DATA_STREAM)
  debugUART.print("Display Manager Running on Core #: ");
  debugUART.println(xPortGetCoreID());
  #endif 
  
  for(;;){  
    
    
    myCount = count;

    if (myCount != oldCount){
      // Total refresh every 10 counts. (Or when we zero out the counter.)
      if ((myCount%10==0)) {
        initDisplay();
        displayCountScreen(myCount);
      }       
      showValue(myCount);  
      oldCount = myCount;
    }
    vTaskDelay(countDisplayUpdateRate / portTICK_PERIOD_MS);

  }
    
}


//****************************************************************************************
// Check for display mode button being pressed and reset the vehicle count
void handleModeButtonPress(){
  // Check for RESET button being pressed. If it has been, reset the counter to zero.
  if (digitalRead(CTR_RESET)== LOW) { 
    count = 0;
    clearLIDARDistanceHistogram();
    #if !(SHOW_DATA_STREAM)
       debugUART.print("Loop: RESET button pressed. Count: ");
       debugUART.println(count);  
    #endif
    //initDisplay();
    //displayCountScreen(count);
    //showValue(count);
  }  
}


//****************************************************************************************
// Device initialization                                   
//****************************************************************************************
void setup(){


  String hwStatus = "";             // String to hold the results of self-test
  
  initPorts();                      // Set up UARTs and GPIOs
  initUI();                         // Splash screens 
  initJSONConfig(filename, config); // Setup SD card and load default values.

  initDisplay();
  displaySplashScreen("(LIDAR Counter)",SW_VERSION); 
  
  // If the unit is unconfigured or is booted witht the RESET button held down, enter AP mode.
  if ((config.deviceName == "YOUR_DEVICE_NAME")||(digitalRead(CTR_RESET)== LOW)) {
    
    accessPointMode = true;
    
    debugUART.println("*******************************");
    debugUART.println("Launching in Access Point Mode!");  
    debugUART.println("*******************************");
    debugUART.println("Setting AP (Access Point)…");
    
    WiFi.softAP(ssid);
  
    IPAddress IP = WiFi.softAPIP();
    debugUART.print("  AP IP address: ");
    debugUART.println(IP);
    
    server.begin();

    
    displayAPScreen(ssid, WiFi.softAPIP().toString()); 
    
  } else {
    
    delay(1000);

    #if USE_LORA  
      #if STAND_ALONE_LORA
        debugUART.println("Stand-Alone Mode. Setting AP (Access Point)…");  
        WiFi.softAP(ssid);
        IPAddress IP = WiFi.softAPIP();
        debugUART.print("  AP IP address: ");
        debugUART.println(IP);   
        server.begin();
        displayAPScreen(ssid, WiFi.softAPIP().toString());
        delay(2000); 
      #else
        setLowPowerMode(); // Lower clock rate and turn off WiFi   
      #endif
      
      initLoRa();
      if (configureLoRa(config)){ // Configure radio params  
        hwStatus+="   LoRa:  OK\n\n";
      } else{
        hwStatus+="   LoRa:  ERROR!\n\n";  
      }
    #endif

    #if USE_WIFI
      myMACAddress = getMACAddress();
      enableWiFi(config);
      hwStatus += "   WiFi:  OK\n\n";
      server.begin();
    
    #endif
    
    if (initLIDAR(true)) { // Turn on LIDAR sensor
      hwStatus+="   LIDAR: OK\n\n";
    } else {
      hwStatus+="   LIDAR: ERROR!\n\n";
    }
    
    if (initRTC()) { // Check RTC is present
      hwStatus+="   RTC  : OK\n";
    }else{
      hwStatus+="   RTC  : ERROR!\n";
    }
    
    displayStatusScreen(hwStatus); // Show results of the configuration above
    delay(3000);

    count=0;
    displayCountScreen(count);
    showValue(count);
    delay(1000);

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

  }    
  currentTime = getRTCTime(); 
  debugUART.println();
  debugUART.println("RUNNING!");  
  debugUART.println(); 
  
}


//****************************************************************************************
// Main Loop
//****************************************************************************************

long lastHistMS = 0; // Timer variable

void loop(){ 

  // Grab the current time
  currentTime = getRTCTime();
    
  //**************************************************************************************
  // Access Point (AP) Operation
  //**************************************************************************************  
  
  if (accessPointMode){ 

    WiFiClient client = server.available();        // Listen for incoming clients
    
    if (client){  // Handle the web UI
      processWebClient("counter", client, config); // Tweak and save parameters
    } 
    
  //**************************************************************************************
  // Standard Operation  
  //**************************************************************************************
  } else { 

    #if (USE_WIFI || STAND_ALONE_LORA)
      WiFiClient client = server.available();        // Listen for incoming clients
    
      if (client){  // Handle the web UI
        processWebClient("counter", client, config); // Tweak and save parameters
       } 
    #endif

         
    // Test if vehicle event has occured 
    // TODO: Consider putting this in a separate RTOS Task. 
      vehicleMessageNeeded = processLIDARSignal(config); // Threshold Detection Algorithm

    // Check for display mode button being pressed and switch display
      handleModeButtonPress();

    //********************************************************************************
    // Check if we need to send a message of some kind
    //********************************************************************************

    //********************************************************************************
    // Boot messages are sent at startup.
      if (bootMessageNeeded){
        msgPayload = buildJSONHeader("b",count);
        msgPayload = msgPayload + "}"; 
        jsonPostNeeded = true;
        bootMessageNeeded = false;
      }

    //********************************************************************************
    // Issue a heartbeat message every hour.
      heartbeatMinute = getRTCMinute();
      if ((oldheartbeatMinute != bootMinute) && (heartbeatMinute == bootMinute)){heartbeatMessageNeeded = true;}  
      if (heartbeatMessageNeeded){
        msgPayload = buildJSONHeader("hb",count);
        msgPayload = msgPayload + "}";
        jsonPostNeeded = true;
        heartbeatMessageNeeded = false; 
      }
      oldheartbeatMinute = heartbeatMinute;

    //********************************************************************************
    // Issue a vehicle event message
      if (vehicleMessageNeeded){ 
        if (lidarBuffer.size()==lidarSamples){ // Fill up the buffer before processing so 
                                               // we don't get false events at startup.      
          count++;     
          #if !(SHOW_DATA_STREAM)
            debugUART.print("Vehicle event! Counts: ");
            debugUART.println(count);  
          #endif             
          msgPayload = buildJSONHeader("v",count);

           
          #if (USE_WIFI) && (APPEND_RAW_DATA_WIFI)
            // Vehicle passing event messages may include raw data from the sensor.
            // If so, tack the data buffer on the JSON message.
            msgPayload = msgPayload + ",\"rawSignal\":[";
            using index_t = decltype(lidarBuffer)::index_t;
            for (index_t i = 0; i < lidarBuffer.size(); i++) {
              msgPayload = msgPayload + lidarBuffer[i]; 
              if (i<lidarBuffer.size()-1){ 
                msgPayload = msgPayload + ",";
              }
            }
            msgPayload = msgPayload + "]";
          #endif
          
          msgPayload = msgPayload + "}";   
          jsonPostNeeded = true; 
        }
        vehicleMessageNeeded = false; 
      }
    

    //********************************************************************************
    // If we need to send a message, push the JSON payload into the message buffer. 
    // The messageManager task will handle it in a separate task. 
      if (jsonPostNeeded){
        xSemaphoreTake(mutex_v, portMAX_DELAY);  
          msgBuffer.push(msgPayload);
        xSemaphoreGive(mutex_v);
        jsonPostNeeded = false;     
      }  
  } 
}
