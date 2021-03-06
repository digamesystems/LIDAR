//---------------------------------------------------------------------------------------------
/* TEMPERATURE LOGGER

   Copyright 2021, Digame Systems. All rights reserved.
*/
//---------------------------------------------------------------------------------------------

#define debugUART Serial
#define drokUART  Serial1

#define USE_WIFI true     // Use WiFi as the reporting link

#if USE_WIFI
String model = "DS-T-LOGGER-WIFI-1";
String model_description = "(Temperature Logger with WiFi Back Haul)";
#endif 
//---------------------------------------------------------------------------------------------

#include <digameDebug.h>      // Debug message handling.
#include <digameVersion.h>    // Global SW version constants
#include <digameJSONConfig.h> // Read program parameters from config file on SD card. 
                              //   TODO: Re-think this strategy. Too much coupling.
                              //   The config struct is used all over the place.
#include <digameTime.h>       // Time Functions - RTC, NTP Synch etc
#include <digameNetwork.h>    // Network Functions - Login, MAC addr
#include <digamePowerMgt.h>   // Power management modes 
#include <digameDisplay.h> //

unsigned long count = 0;      // The number of vehicle events recorded
#include <digameCounterWebServer.h>  // Web page to tweak parameters. Uses count. TODO: Fix.

#include <CircularBuffer.h>   // Adafruit library for handling circular buffers of data. 
// https://github.com/rlogiacco/CircularBuffer

//---------------------------------------------------------------------------------------------


const int samples = 200;      // The number of messages we'll buffer
CircularBuffer<String *, samples> msgBuffer; // The buffer containing pointers JSON messages to

// be sent to the or server.
String msgPayload;            // The message being sent to the base station.
// (JSON format depends on link type: LoRa or WiFi)

// Access point mode
bool accessPointMode = false; //
bool usingWiFi = false;       // True if USE_WIFI or AP mode is enabled

// Multi-Tasking
SemaphoreHandle_t mutex_v;        // Mutex used to protect variables across RTOS tasks.
TaskHandle_t messageManagerTask;  // A task for handling data reporting
TaskHandle_t displayManagerTask;  // A task for updating the EInk display

// Messaging flags
bool   jsonPostNeeded         = false;
bool   bootMessageNeeded      = true; // Send boot and heartbeat messages at startup
bool   heartbeatMessageNeeded = true;
int    vehicleMessageNeeded   = 0;    // 0=false, 1=lane 1, 2=lane

// Over the Air Updates work when we have WiFi or are in Access Point Mode...
bool useOTA = true;

// DIO
int    LED_DIAG  = 12;     // Indicator LED
int    CTR_RESET = 32;     // Counter Reset Input

// Bookeeping
String myMACAddress;       // Read at boot
unsigned long bootMillis = 0; // When did we boot according to millis()? Used to calculate uptime.
int    bootMinute;         // The minute (0-59) within the hour we woke up at boot.
int    heartbeatMinute;    // We issue Heartbeat message once an hour. Holds the minute we do it.
int    oldheartbeatMinute; // Value the last time we looked.
unsigned long lastHeartbeatMillis = 0; // millis() value of the last hearbeat
String currentTime;        // Set in the main loop from the RTC
int    currentSecond;      // Set in the main loop from the RTC

float currentTemperature;

bool wifiMessagePending = true;

//---------------------------------------------------------------------------------------------

// Used in setup()
void loadConfiguration(String &statusMsg);

void showSplashScreen();

void configurePorts(String &statusMsg);
void configureEinkDisplay(String &statusMsg);
void configureAPMode(String &statusMsg);
void configureStationMode(String &statusMsg);
void configureRTC(String &statusMsg);
void configureNetworking(String &statusMsg);
void configureCore0Tasks(String &statusMsg);
void configureTimers(String &statusMsg);

// Used in loop()
void handleBootEvent();       // Boot messages are sent at startup.
void handleResetEvent();      // Look for reset flag getting toggled
void handleModeButtonPress(); // Check for display mode button being pressed and switch display
void handleVehicleEvent();    // Read the LIDAR sensor and enque a count event msg, if needed
void handleHeartBeatEvent();  // Check timers and enque a heartbeat event msg, if needed


//****************************************************************************************
void setup() // DEVICE INITIALIZATION
//****************************************************************************************
{
  String statusMsg = "";         // String to hold the results of self-test

  configurePorts(statusMsg);     // Set up UARTs and GPIOs
  showSplashScreen();
  configureEinkDisplay(statusMsg);
  loadParameters(statusMsg);     // Grab program settings from SD card
  
  configureNetworking(statusMsg);

  configureRTC(statusMsg);         // Configure RTC. Set it if NTP is available.

  displayStatusScreen(statusMsg);  // Show the results of the hardware configuration
  delay(5000);

  displayCountScreen(count);       // Set up the display to show vehicle events
  showValue(count);
  delay(1000);

  configureCore0Tasks(statusMsg);  // Set up eink display and message tasks on core 0

  configureTimers(statusMsg);      // intitialize timer variables

  DEBUG_PRINTLN("RUNNING!\N");
  
}

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

//****************************************************************************************
void loop() // MAIN LOOP
//****************************************************************************************
{
  unsigned long T1, T2;

  T1 = millis(); // Time at the start of the loop.

  currentTime   = getRTCTime();
  currentSecond = getRTCSecond();
  //config.temperature = String(getRTCTemperature(),1);

  handleBootEvent();       // Boot messages are sent at startup.
  handleResetEvent();      // Look for reset flag getting toggled.
  handleModeButtonPress(); // Check for display mode button being pressed and switch display
  //handleVehicleEvent();    // Read the LIDAR sensor and enque a count event msg, if needed
  handleHeartBeatEvent();  // Check timers and enque a heartbeat event msg, if needed

  // Tune loop to run at about 50Hz
  if (wifiConnected){ // 80Mhz clock
    delay(11);
  } else {
    delay(6);         // 40Mhz clock
  }
  String s = "";
  if (drokUART.available()){
     s = drokUART.readStringUntil('\n');
     debugUART.println(s);
      config.temperature = getValue(s,',',0);
      config.temperature.trim();
      
      int len1 = config.temperature.length();
      if (len1>=2){ // Remove garbage characters from controller message. 
        debugUART.println("T string len: " + String(len1));
        config.temperature = config.temperature.substring(0,len1-2);
      }
      
      config.relay = getValue(s,',',1);
      config.relay.trim();
      debugUART.println("T: " + config.temperature);
      debugUART.println("R: " + config.relay);
     //
  }

  //debugUART.println(millis());
  
  T2 = millis();
  //DEBUG_PRINTLN(T2-T1);

}

//**************************************************************************************
void loadParameters(String &statusMsg)
//**************************************************************************************
{
  // Setup SD card and load default values.
  if (initJSONConfig(filename, config))
  {
    statusMsg += "   SD   : OK\n\n";
  } else {
    statusMsg += "   SD   : ERROR!\n\n";
  };
}

//**************************************************************************************
void configureTimers(String &statusMsg) {
  bootMillis          = millis();
  upTimeMillis        = millis() - bootMillis;
  lastHeartbeatMillis = millis();
  currentTime         = getRTCTime();
  bootMinute          = getRTCMinute();
}

//**************************************************************************************
void configureCore0Tasks(String &statusMsg) {
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
}

//**************************************************************************************
void configureNetworking(String &statusMsg) {

  // If the unit is unconfigured or is booted with the RESET button held down, enter AP mode.
  // Recently added a check to see if the unit was booted with something in front of the
  // sensor. If so, come up in AP mode. -- This way folks don't have to press the button at
  // boot.
  accessPointMode = ( (config.deviceName == "YOUR_DEVICE_NAME") ||
                      (digitalRead(CTR_RESET) == LOW)
                    );

  if (accessPointMode) {

    setMediumPowerMode(); // Slow Down CPU and turn off Bluetooth. See: DigamePowerMangement.h
    useOTA = true;
    usingWiFi = true;
    configureAPMode(statusMsg);
    myMACAddress = getMACAddress();

  } else {

  // Data reporting over the WiFi link
#if USE_WIFI
    useOTA = true;
    usingWiFi = true;
    setMediumPowerMode(); // Slow Down CPU and turn off Bluetooth. See: DigamePowerMangement.h
    myMACAddress = getMACAddress();
    configureStationMode(statusMsg);
#endif
  
    }

  if (usingWiFi) {
    DEBUG_PRINTLN("  Initializing Web Server...");
    initWebServer();
    http.setReuse(true); // See digameNetwork.h for this guy.
  }

  DEBUG_PRINT("  USE OTA: ");
  DEBUG_PRINTLN(useOTA);

}

//**************************************************************************************
void configureRTC(String &statusMsg) {
  // Check that the RTC is present
  if (initRTC()) {
    statusMsg += "   RTC  : OK\n";
  } else {
    statusMsg += "   RTC  : ERROR!\n";
  }

  // Synchronize the RTC to an NTP server, if available
#if USE_WIFI
  if (wifiConnected) { // Attempt to synch ESP32 clock with NTP Server...
    synchTimesToNTP();
  }
#endif

  DEBUG_PRINTLN();
}


//****************************************************************************************
void configureStationMode(String &statusMsg) {
  enableWiFi(config);
  displayIPScreen(String(WiFi.localIP().toString()));
  delay(3000);
  statusMsg += "   WiFi : OK\n\n";
}

//****************************************************************************************
// Configure the device as an access point. TODO: move to DigameNetwork.h
void configureAPMode(String &statusMsg) {
  const char* ssid = (String("Digame-CTR-") + getShortMACAddress()).c_str();
  DEBUG_PRINTLN("  Stand-Alone Mode. Setting AP (Access Point)???");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid);
  IPAddress IP = WiFi.softAPIP();
  DEBUG_PRINT("    AP IP address: ");
  DEBUG_PRINTLN(IP);
  displayAPScreen(ssid, WiFi.softAPIP().toString());
  delay(5000);
}

//**************************************************************************************
void configureEinkDisplay(String &statusMsg) {
  initDisplay();
  displaySplashScreen("(Temp Logger)", SW_VERSION);
}

//****************************************************************************************
// Prepare the GPIOs and debugUART serial port. Bring up the Wire interface for SPI bus.
void configurePorts(String &statusMsg) {
  // Ready the LED and RESET pins
  pinMode(LED_DIAG, OUTPUT);
  pinMode(CTR_RESET, INPUT_PULLUP);
  debugUART.begin(115200);   // Intialize terminal serial port
  delay(1000);               // Give port time to initalize

  drokUART.begin(9600, SERIAL_8N1, 25, 33);
  delay(1000);
  debugUART.println("Sending the Start Message...");
  
  drokUART.println("start");
  
  Wire.begin();
}

//****************************************************************************************
void showSplashScreen() {
  String compileDate = F(__DATE__);
  String compileTime = F(__TIME__);

  DEBUG_PRINTLN("***************************************************************");
  DEBUG_PRINTLN("              DIGAME Temperature Logger");
  DEBUG_PRINTLN("                        ---");
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

//**************************************************************************************
// Add a message to the queue for transmission.
void pushMessage(String message) {
  xSemaphoreTake(mutex_v, portMAX_DELAY);

  #if USE_WIFI
    if (accessPointMode == false) {
      String * msgPtr = new String(message);
      msgBuffer.push(msgPtr);
    }
  #endif
    xSemaphoreGive(mutex_v);
}

//****************************************************************************************
// WiFi can handle a more human-readable JSON data payload.
String buildWiFiJSONHeader(String eventType, double count, String lane = "1") {
  String jsonHeader;
  String strCount;

  strCount = String(count, 0);
  strCount.trim();

  eventType = "v"; 
  if (eventType == "b") eventType  = "Boot";
  if (eventType == "hb") eventType = "Heartbeat";
  if (eventType == "v") eventType  = "Vehicle";

  jsonHeader = "{\"deviceName\":\""      + config.deviceName +
               "\",\"deviceMAC\":\""   + myMACAddress +      // Read at boot
               "\",\"firmwareVer\":\"" + TERSE_SW_VERSION  +
               "\",\"timeStamp\":\""   + currentTime +       // Updated in main loop from RTC
               "\",\"eventType\":\""   + eventType +
               "\",\"count\":\""       + String((config.relay=="OP")) + //strCount +          // Overloading the count with the relay state
               "\",\"rssi\":\""        + String(getRTCTemperature()*10,2) + //strCount +      // Overloading the RSSI with the RTC temp
               
               "\",\"temp\":\""        + config.temperature; // String(getRTCTemperature(), 1); // Thermistor Temperature in C

  if (eventType == "Vehicle") {
    jsonHeader = jsonHeader +
                 "\",\"detAlgorithm\":\"" + "Threshold";// Detection algorithm (Threshold)
    jsonHeader = jsonHeader +
                 "\",\"lane\":\"" + lane + "\"";        // Lane in which the vehicle was seen
  }

  if ((eventType == "Boot") || (eventType == "Heartbeat")) { //In the boot/heartbeat messages, send the current settings.
    
    /*
    jsonHeader = jsonHeader +
                 "\",\"settings\":{" +
                 " \"time\":\"" + currentTime  + "\"" +
                 ",\"relay\":\"" + String((config.relay=="relay:OP"))  + "\"" +
                 ",\"ctlTemp\":\"" + String(getRTCTemperature())  + "\"" +
                 "}";
    */
    jsonHeader = jsonHeader +
                 "\",\"rawSignal\":[" +
                 String((config.relay=="OP")) +
                 "," + String((config.relay=="OP")) +
                 ","  + String(getRTCTemperature())  + "]";
  }

  //  jsonHeader = jsonHeader + "\"";

  return jsonHeader;
}

//****************************************************************************************
// JSON messages to the server all have a similar format. This builds the common header.
String buildJSONHeader(String eventType, double count, String lane = "1") {
  String retValue = "";

#if USE_WIFI
  return buildWiFiJSONHeader(eventType, count, lane);
#endif

}

//****************************************************************************************
/* Playing around with scheduling message delivery to minimize interference between LoRa
    counters.

    Divide the minute up into equal portions for the number of counters
    we are dealing with. Each counter has his own window within the minute to transmit.

    TODO: Think about having the base station tell us
    how many friends we have in the area and which counter we are...

    counterNumber = 1 to 4
    numCounters   = 1 to 4
*/
bool inTransmitWindow(int counterNumber, int numCounters = 3) {

  int thisSecond;
  thisSecond = currentSecond;  // Set in the main loop from the RTC

  // return true; // Uncomment to turn off time window check and allow counters to respond at any time.

  if (config.showDataStream == "false") {
    DEBUG_PRINT("This Second: "); 
    DEBUG_PRINTLN(thisSecond);
  }

  int windowInterval = 60 / (numCounters);
  // EX: for four counters, we get windows of...
  // counter 0: 0-14,
  // counter 1:      15-29,
  // counter 2:            30-44,
  // counter 3:                  45-59

  if ( (thisSecond >=   (counterNumber - 1) * windowInterval ) &&
       (thisSecond <  ( (counterNumber - 1) * windowInterval + windowInterval ) )
     )
  {
    return true;
  } else {
    return false;
  }
}


//****************************************************************************************
// A task that runs on Core0 using a circular buffer to enqueue messages to the server...
// Current version keeps retrying forever if an ACK isn't received. TODO: Fix.
void messageManager(void *parameter) {

  bool messageACKed = true;

  DEBUG_PRINT("Message Manager Running on Core #: ");
  DEBUG_PRINTLN(xPortGetCoreID());
  DEBUG_PRINTLN();

  for (;;) {

    //*******************************
    // Process a message on the queue
    //*******************************
    if ( (msgBuffer.size() > 0) &&
         (inTransmitWindow(config.counterID.toInt(), config.counterPopulation.toInt())) )
    {
      
      wifiMessagePending = true;
        
      if (config.showDataStream == "false") {
        DEBUG_PRINT("Buffer Size: ");
        DEBUG_PRINTLN(msgBuffer.size());
      }

      String activeMessage = String(msgBuffer.first()->c_str()); // Read from the buffer without removing the data from it.

      // Send the data directly to the ParkData server via http(s) POST
      #if USE_WIFI
        messageACKed = postJSON(activeMessage, config);
      #endif  

      if (messageACKed)
      {
        // Message sent and received. Take it off of the queue.
        xSemaphoreTake(mutex_v, portMAX_DELAY);
        String  * entry = msgBuffer.shift();
        delete entry;
        xSemaphoreGive(mutex_v);

        if (config.showDataStream == "false")
        {
          DEBUG_PRINTLN("Success!");
          DEBUG_PRINTLN();
          wifiMessagePending = false;
      
        }
      } else {
        if (config.showDataStream == "false")
        {
          DEBUG_PRINTLN("******* Timeout Waiting for ACK **********");
          DEBUG_PRINT("Retrying...");
        }
      }
    } else {
      #if USE_WIFI
        if (wifiConnected) {
          if (!accessPointMode) { // Don't turn off WiFi if we are in accessPointMode
            /*
            unsigned long timeOut = 120000; 
            unsigned long tNow    = millis();
            
            // Turn WiFi off after an interval of inactivity to save power
            
            if ( ((tNow - msLastPostTime)         > timeOut) && 
                 ((tNow - msLastWebPageEventTime) > timeOut) )
            { 
              DEBUG_PRINTLN("WiFi Stale...");
              disableWiFi();
              wifiMessagePending = false;
            }
            */
          }
        }
      #endif
    }


    vTaskDelay(100 / portTICK_PERIOD_MS);

  }
}


//****************************************************************************************
// Text-based "I'm alive" indicator that we stick on the eInk display and update
// periodically to show the program is running.
String rotateSpinner() {
  static String spinner = "|";

  //OLD SCHOOL! :)
  if (spinner == "|") {
    spinner = "/";
  } else if (spinner == "/") {
    spinner = "-";
  } else if (spinner == "-") {
    spinner = "\\";
  } else {
    spinner = "|";
  }
  return spinner;
}

//****************************************************************************************
// A task that runs on Core0 to update the display when the count changes.
void countDisplayManager(void *parameter) {
  int countDisplayUpdateRate = 200;
  static unsigned int oldCount = 0;

  if (config.showDataStream == "false") {
    DEBUG_PRINT("Display Manager Running on Core #: ");
    DEBUG_PRINTLN(xPortGetCoreID());
    DEBUG_PRINTLN();
  }

  for (;;) {
    if (count != oldCount) {
      // Total refresh every 100 counts. (Or when we zero out the counter.)
      if (count % 100 == 0) {
        initDisplay();
        displayCountScreen(count);
      }
      showValue(count);
      oldCount = count;
    }
    showPartialXY(rotateSpinner(), 180, 180);
    upTimeMillis = millis() - bootMillis; //TODO: Put this somewhere else.
    vTaskDelay(countDisplayUpdateRate / portTICK_PERIOD_MS);
  }
}

//***************************************************************************************
// Check for display mode button being pressed and reset the vehicle count
void handleModeButtonPress() {
  // Check for RESET button being pressed. If it has been, reset the counter to zero.
  if (digitalRead(CTR_RESET) == LOW) {
    count = 0;
    if (config.showDataStream == "false") {
      DEBUG_PRINT("Loop: RESET button pressed. Count: ");
      DEBUG_PRINTLN(count);
    }
  }
}

//**************************************************************************************
void handleBootEvent() {
  if (bootMessageNeeded) {
    msgPayload = buildJSONHeader("b", count);
    msgPayload = msgPayload + "}";
    pushMessage(msgPayload);
    if (config.logBootEvents == "checked") {
      appendTextFile("/eventlog.txt", msgPayload);
    }
    bootMessageNeeded = false;
  }
}


//**************************************************************************************
void handleResetEvent() {
  if (resetFlag) {
    DEBUG_PRINTLN("Reset flag has been flipped. Rebooting the processor.");
    delay(1000);
    ESP.restart();
  }
}

//**************************************************************************************
void handleHeartBeatEvent() { // Issue a heartbeat message, if needed.

  unsigned long deltaT = (millis() - lastHeartbeatMillis);
  unsigned long slippedMilliSeconds = 0;

  //debugUART.println(deltaT);
  
  if ( (deltaT) >= config.heartbeatInterval.toInt() * 1000 ) {
    if (config.showDataStream == "false") {
      DEBUG_PRINT("millis: ");
      DEBUG_PRINTLN(deltaT);
    }
    slippedMilliSeconds = deltaT - config.heartbeatInterval.toInt() * 1000; // Since this Task is on a 100 msec schedule, we'll always be a little late...
    if (config.showDataStream == "false") {
      DEBUG_PRINT("slipped ms: ");
      DEBUG_PRINTLN(slippedMilliSeconds);
    }
    heartbeatMessageNeeded = true;
  }

  if (heartbeatMessageNeeded) {
    msgPayload = buildJSONHeader("hb", count);
    msgPayload = msgPayload + "}";
    pushMessage(msgPayload);
    if (config.logHeartBeatEvents == "checked") {
      appendTextFile("/eventlog.txt", msgPayload);
    }
    heartbeatMessageNeeded = false;
    lastHeartbeatMillis = millis() - slippedMilliSeconds;
  }
}
