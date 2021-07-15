/*
 * HEIMDALL VCS - Vehicle Counting Systems 
 * A traffic counting system that uses LIDAR to track pedestrians and vehicles.
 * 
 * Copyright 2021, Digame Systems. All rights reserved.
 */

// Mix-n-match hardware configuration
#define ADAFRUIT_EINK_SD true   // Some Adafruit Eink Displays have an integrated SD card so we don't need a separate module
#define SHOW_DATA_STREAM false  // A debugging flag to show raw LIDAR values on the serial monitor
 
#include <TFMPlus.h>            // Include TFMini Plus LIDAR Library v1.4.0
#include <WiFi.h>               // WiFi stack
#include <CircularBuffer.h>     // Adafruit library. Pretty small!

#include "digameTime.h"         // Digame Time Functions
//#include "digameMath.h"         // Mean and correlation
#include "digameJSONConfig.h"   // Program parameters from config file on SD card
#include "digameNetwork.h"      // Digame Network Functions
#include "digameDisplay.h"    // Digame eInk Display Functions.


#include "driver/adc.h"
#include <esp_bt.h>

// Aliases for easier reading
#define debugUART  Serial
#define tfMiniUART Serial2  
#define LoRaUART Serial1


String swVersion = "0.9.3";
bool accessPointMode = false;
// Set web server port number to 80
WiFiServer server(80);
const char *ssid = "Digame-AP";

Config config; // A structure to hold program configuration parameters. See: digameJSONConfig.h

const int samples = 100;
CircularBuffer<int, samples> buffer; // We're going to hang onto the last 100 points to visualize what the sensor sees
CircularBuffer<String, samples> loraMsgBuffer; // A buffer containing JSON messages to be sent to the LoRa basestation.
float data[samples];

SemaphoreHandle_t mutex_v; // Mutex used to protect our jsonMsgBuffers (see below).

TFMPlus tfmP;           // Create a TFMini Plus object

// HW Status Variables -- sniffed in setup()
bool sdCardPresent = false;
bool rtcPresent    = false;
bool lidarPresent  = false; 
bool wifiConnected = false;

long msLastConnectionAttempt;  // Timer value of the last time we tried to connect to the wifi.

int LED_DIAG  = 12;   // Indicator LED
int CTR_RESET = 32;   // Counter Reset Input

int heartbeatTime;    // Issue Heartbeat message once an hour. Holds the current Minute.
int oldHeartbeatTime; // Value the last time we looked.

// Messaging flags
bool jsonPostNeeded         = false;
bool bootMessageNeeded      = true;
bool heartbeatMessageNeeded = false;
bool vehicleMessageNeeded   = false;

// The message being sent
String loraPayload; //A stripped down version for LoRa

// The minute (0-59) within the hour we woke up at boot. 
int bootMinute;

String hwStatus="";

//****************************************************************************************
void disableWiFi(){
    adc_power_off();
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF);    // Switch WiFi off
    //debugUART.println("");
    //debugUART.println("WiFi disconnected!");
}

//****************************************************************************************
void disableBluetooth(){
    // Rather disappointing on power improvement.
    btStop();
    //debugUART.println("");
    //debugUART.println("Bluetooth stop!");
}
 

//****************************************************************************************
void setLowPowerMode() {
    debugUART.print("  Setting Power Mode... ");
    disableWiFi();
    disableBluetooth();
    setCpuFrequencyMhz(40);
    debugUART.println(" Done. (Low Power Mode Enabled)");
}


//****************************************************************************************
// LoRa messages to the server all have a similar format. 

String buildLoRaHeader(String eventType, double count){
  String loraHeader;
  String strCount;

  strCount= String(count,0);
  strCount.trim();
  if (rtcPresent){
    loraHeader = "{\"ts\":\"" + getRTCTime() + 
                   "\",\"et\":\"" + eventType +
                   "\",\"c\":\"" + strCount +  
                   "\",\"t\":\"" + String(getRTCTemperature(),1)+
                   "\"";
  } else {
    loraHeader = "{\"ts\":\"" + getLocalTime() + 
                   "\",\"et\":\"" + eventType + 
                   "\",\"c\":\"" + strCount + 
                   "\",\"t\":\"" + String(getRTCTemperature(),1)+
                   "\"";  
  }                    
  return loraHeader;
}


//************************************************************************
// LIDAR Siganal Processing Functions
//************************************************************************

//************************************************************************
// A dirt-simple processing scheme based on a hard threshold read from the
// SD card. Pretty susceptible to noisy conditions. TODO: Improve. 
bool processLIDARSignal(){
  // LIDAR signal analysis parameters
  
    int16_t tfDist        = 0;    // Distance to object in centimeters
    int16_t tfFlux        = 0;    // Strength or quality of return signal
    int16_t tfTemp        = 0;    // Internal temperature of Lidar sensor chip
    static float smoothed = 0.0;
    
    static bool  carPresent      = false;    
    static bool  lastCarPresent  = false; 
    int   carEvent        = 0;
    int   lidarUpdateRate = 20;
    float correl1 = 0.0;
  
    bool retValue = false;

    tfmP.sendCommand(TRIGGER_DETECTION, 0); //Uncomment in triggered mode.
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
      smoothed = smoothed * (config.lidarSmoothingFactor.toFloat()) + (float)tfDist * (1.0- config.lidarSmoothingFactor.toFloat());
      int intSmoothed = (int) smoothed*10;

      buffer.push(intSmoothed);
      
      if (smoothed < config.lidarDistanceThreshold.toFloat()) {
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
     debugUART.print(config.lidarDistanceThreshold.toFloat());
     debugUART.print(",");
     debugUART.println(carEvent);
#endif
     
     lastCarPresent = carPresent;
  }

    
  return retValue;  
}



//****************************************************************************************
// Sends a message to another LoRa module and listens for an ACK reply.
bool sendReceiveLoRa(String msg){
  long timeout = 10000;
  long t2,t1;

  t1 = millis();
  t2 = t1;

  bool replyPending = true;
  
  //Send the message... Base Stations Default to an address of 1.
  String reyaxMsg = "AT+SEND=1,"+String(msg.length())+","+msg;

  #if SHOW_DATA_STREAM
  #else
    debugUART.print("Sending LoRa Message: ");
    debugUART.println(reyaxMsg);
  #endif
  
  LoRaUART.println(reyaxMsg);
  
  //wait for ACK or timeout
  while ((replyPending == true) && ((t2-t1)<timeout)){
    t2=millis();
    if (LoRaUART.available()) {
      String inString = LoRaUART.readStringUntil('\n');
      if (replyPending) {
        if (inString.indexOf("ACK")>=0){
          replyPending = false;
          #if SHOW_DATA_STREAM
          #else
            debugUART.println("ACK Received: " + inString); 
          #endif
          return true;
        }
      } 
    } 
  }
  
  if((t2-t1)>= timeout){
    #if SHOW_DATA_STREAM
    #else
      debugUART.println("Timeout!");
    #endif
    return false;
  }

  vTaskDelay(1000 / portTICK_PERIOD_MS); 
  
}




/********************************************************************************/
// Experimenting with using a circular buffer to enqueue messages to the server...
void messageManager(void *parameter){
  String activeMessage;
  
  for(;;){  

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

      //Send the data to the base station
      //sendReceive("AT+MODE=0");
      while (!sendReceiveLoRa(activeMessage)){};
      //sendReceive("AT+MODE=1");
      #if SHOW_DATA_STREAM
      #else
        Serial.println("Sent!");
      #endif
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS);   
  }   
}

//************************************************************************
String sendReceive(String s){
  //String strReturn;
  String loraMsg;

  //Send the command
  LoRaUART.println(s);

  //Read reply
  while (!LoRaUART.available()) {
    delay(10);
  }
  loraMsg = LoRaUART.readStringUntil('\n');
  //}

  //debugUART.println(loraMsg);
  return loraMsg;
  
}

//************************************************************************
void configureLoRa(Config config){
  
  debugUART.println("  Configuring LoRa...");
  
  debugUART.println("    Setting Address to: " + config.loraAddress);
  sendReceive("AT+ADDRESS="+config.loraAddress);
  sendReceive("AT+ADDRESS?");
  sendReceive("AT+ADDRESS?");

  debugUART.println("    Setting Network ID to: " + config.loraNetworkID);
  sendReceive("AT+NETWORKID="+config.loraNetworkID);
  sendReceive("AT+NETWORKID?");
  
  debugUART.println("    Setting Band to: " + config.loraBand);
  sendReceive("AT+BAND="+config.loraBand);
  sendReceive("AT+BAND?");

  debugUART.println("    Setting Modulation Parameters to: " + config.loraSF+","+config.loraBW+","+config.loraCR+","+config.loraPreamble);
  sendReceive("AT+PARAMETER="+config.loraSF+","+config.loraBW+","+config.loraCR+","+config.loraPreamble);
  sendReceive("AT+PARAMETER?");

  //sendReceive("AT+MODE=1");
  
  hwStatus+= "   LoRa : OK\n\n";
}



//****************************************************************************************
void splash(){
    String compileDate = F(__DATE__);
    String compileTime = F(__TIME__);
  
    debugUART.println("*****************************************************");
    debugUART.println("ParkData Traffic Monitoring Platform");
    debugUART.println("Version " + swVersion);
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
void initPorts(){
    // Ready the LED and RESET pins
    pinMode(LED_DIAG, OUTPUT);
    pinMode(CTR_RESET,INPUT_PULLUP);      
    debugUART.begin(115200);   // Intialize terminal serial port
    LoRaUART.begin(115200, SERIAL_8N1, 25, 33);
    delay(1000);               // Give port time to initalize
    Wire.begin(); 
}

void initUI(){
    display.init(0);
    // first update should be full refresh
    initDisplay();
    displaySplashScreen(swVersion);
    splash();
    //delay(5000);
    //displayInitializingScreen();
}

void initFileSystem(){
    debugUART.print("  Testing for SD Card Module... ");
    sdCardPresent = initSDCard();
    if (sdCardPresent){
      //saveConfiguration(filename, config);
      loadConfiguration(filename, config);
      
      debugUART.println("Module found. (Parameters read from SD Card.)");
      hwStatus+= "   SD   : OK\n\n";
    }else{
      debugUART.println("ERROR! Module NOT found. (Parameters set to default values.)");    
      hwStatus+= "   SD   : ERROR!\n\n";
    }  
}

void initLIDAR(){
  tfMiniUART.begin(115200);  // Initialize TFMPLus device serial port.
    delay(1000);               // Give port time to initalize
    tfmP.begin(&tfMiniUART);   // Initialize device library object and...
                               // pass device serial port to the object.

    // Send some commands to the TFMini-Plus
    // - - Perform a system reset - - 
    
    debugUART.print( "  Activating LIDAR Sensor... ");
    
    //debugUART.printf( "  Testing for LIDAR Sensor... ");
    if( tfmP.sendCommand(SYSTEM_RESET, 0)){
        debugUART.println("Done. (LIDAR Sensor initialized)");
        hwStatus+="   LIDAR: OK\n\n";

        delay(500);
        
        debugUART.printf( "  Adjusting Frame Rate... ");
        if( tfmP.sendCommand(SET_FRAME_RATE, FRAME_0)){ //FRAME_0 is triggered mode.
           debugUART.println("  Frame Rate Adjusted.");
        }
        else tfmP.printReply(); 
    }
    else {
        debugUART.println("ERROR! LIDAR Sensor not found or sensor error.");
        hwStatus+="   LIDAR: ERROR!\n\n";
        //tfmP.printReply();
    }
  
}

void initRealTimeClock(){
    debugUART.print("  Testing for Real-Time-Clock module... ");
    rtcPresent = initRTC();
    if (rtcPresent){
      debugUART.println("RTC found. (Program will use time from RTC)");
      hwStatus+="   RTC  : OK\n\n";
      debugUART.print("      RTC Time: ");
      debugUART.println(getRTCTime()); 
    }else{
      debugUART.println("ERROR! Could NOT find RTC. (Program will attempt to use NTP time)");   
      hwStatus+="    RTC  : ERROR!\n\n"; 
    }

    bootMinute = getRTCMinute();
    heartbeatTime = bootMinute;
    oldHeartbeatTime = heartbeatTime;
  

}

//****************************************************************************************
//Extract a value for a query parameter from an HTTP header.
String getParam(String header, String paramName){
  String result= ""; 

  int strStart = header.indexOf(paramName+"=");
  
  if (strStart == -1){
    Serial.println(paramName + " not found.");
    return ("Not found.");  
  }
  strStart = strStart + paramName.length() + 1;

  int strStop = header.indexOf(" HTTP/1.1");

  result = header.substring(strStart,strStop);
  result = result.substring(0,result.indexOf("&"));

  // Clean up the string by removing HTML replacements
  result.replace("+"," ");
  result.replace("%3A",":");
  result.replace("%2F","/");
  
  Serial.println(result);
  
  return result;
}

void processClient(WiFiClient client){
  // Variable to store the HTTP request
  String header;
  // Current time
  unsigned long currentTime = millis();
  // Previous time
  unsigned long previousTime = 0; 
  // Define timeout time in milliseconds (example: 2000ms = 2s)
  const long timeoutTime = 2000;

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // Handle any GET queries we know about.
            if (header.indexOf("GET /general") >=0) {
              Serial.println("You are tweaking general parameters.");  
              config.deviceName = getParam(header, "devname");
              saveConfiguration(filename,config);
              
            } else if (header.indexOf("GET /lora") >=0) {
              Serial.println("You are tweaking LoRa parameters."); 
              config.loraAddress   = getParam(header,"address");
              config.loraNetworkID = getParam(header,"networkid");
              config.loraBand      = getParam(header,"band");
              config.loraSF        = getParam(header,"spreadingfactor");
              config.loraBW        = getParam(header,"bandwidth");
              config.loraCR        = getParam(header,"codingrate");
              config.loraPreamble = getParam(header,"preamble");
              saveConfiguration(filename,config);
              //configureLoRa(config);

            } else if (header.indexOf("GET /lidar") >=0) {
              Serial.println("You are tweaking LIDAR parameters."); 
              config.lidarDistanceThreshold   = getParam(header,"distthreshold");
              config.lidarSmoothingFactor = getParam(header,"smoothfactor");
              saveConfiguration(filename,config);
            }
                      
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head>");
            client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            client.println("<style>");
            client.println(" html { font-family: Helvetica; display: inline-block; margin: 0px auto;}");// text-align: center;}");
            client.println(" form { margin:10px auto; width: 300px; background-color: #eeeeee; border: solid; padding: 5px 15px;}");
            // #e84122
            client.println(" h1 { background-color: #ac0014; border: none; color: white; padding: 16px 40px;");  
            client.println("    text-decoration: none; text-align: center; margin: 2px;}");
            client.println(" h2 { margin: auto; text-align: center;}");
            client.println(" h3 { margin: auto; text-align: center;}");
            client.println(" label {width: 250px; text-align: left;clear: both;float:left; margin-right:15px;}");
            client.println(" img { display:block; width:175px; margin-left:auto; margin-right:auto; }");
            client.println(" input {width: 250px;}");
            client.println(" table input {width: 110px;}");
            client.println(" .narrow { width: 50px;)");
            
            //client.println(" .center {  display: block;  margin-left: auto;  margin-right: auto;  width: 175px;}");
            client.println("</style>");
            client.println("</head>");
            
            // Web Page Heading
            client.println("<body><h1>HEIMDALL VCS</h1><br>");
            client.println("<h3><em>A Next-Generation Traffic Counting Platform</em></h3>");
            client.println("<h3><em>from D&#237game Systems</em></h3>");
            client.println("<hr>");

            client.println("<br><H2>TRAFFIC COUNTER<br><em>"+ config.deviceName +"</em></H2>");
            client.println("<p>Welcome. You can use these forms to update system parameters for this LoRa Sensor.</p>");

            client.println("<form action=\"/generalparms\">\n");
            client.println("<H3>General</H3>");
            
            client.println("<label for=\"devname\">Device Name</label>");
            client.println("<input type=\"text\" id=\"devname\" name=\"devname\" value=\""+ config.deviceName + "\"><br><br>");
            
            client.println("<label>MAC Address: " + getMACAddress() + "</label><br><br>");
            
            client.println("<label>Model<br>");
            client.println("DS-VC-LIDAR-LOR-1<br></label><p><small>(LIDAR Traffic Counter with LoRa Back Haul)</small></p>");
            
            client.println("<input type=\"submit\" value=\"Submit\"></form>");

            client.println("<form action=\"/loraparms\">\n");
            client.println("<H3>LoRa (Low-Power Long-Range Wireless Link)</H3>");
            client.println("<p>Strategies for optimizing the parameters for LoRa communication may be found in the product manual. <TODO: LINK> Both the counters and the base station must use the same RF parameters for communication.</p>");
            client.println("<label for=\"address\">Address</label>");
            client.println("<input type=\"text\" id=\"address\" name=\"address\" value=\""+ config.loraAddress + "\"><br><br>");
            
            client.println("<label for=\"networkid\">Network ID</label>");
            client.println("<input type=\"text\" id=\"networkid\" name=\"networkid\" value=\"" + config.loraNetworkID +"\"><br><br>");
            
            client.println("<label for=\"band\">Band</label>");
            client.println("<input type=\"text\" id=\"band\" name=\"band\" value=\""+ config.loraBand +"\"><br><br>");
            
            client.println("<label for=\"spreadingfactor\">Spreading Factor</label>");
            client.println("<input type=\"text\" id=\"spreadingfactor\" name=\"spreadingfactor\" value=\""+ config.loraSF + "\"</input><br><br>");
            
            client.println("<label for=\"bandwidth\">Bandwidth</label>");
            client.println("<input type=\"text\" id=\"bandwidth\" name=\"bandwidth\" value=\""+ config.loraBW +"\"><br><br>");
            
            client.println("<label for=\"codingrate\">Coding Rate</label>");
            client.println("<input type=\"text\" id=\"codingrate\" name=\"codingrate\" value=\"" + config.loraCR + "\"><br><br>");
            
            client.println("<label for=\"preamble\">Preamble</label>");
            client.println("<input type=\"text\" id=\"preamble\" name=\"preamble\" value=\""+ config.loraPreamble +"\"</input><br><br>");
            client.println("<input type=\"submit\" value=\"Submit\"></form>");


            client.println("<form action=\"/lidarparms\">\n");
            client.println("<H3>LIDAR Detection Parameters</H3>");
            client.println("<p>Strategies for optimizing the parameters for LIDAR vehicle detection may be found in the product manual. <TODO: LINK></p>");
            client.println("<label for=\"address\">Distance Threshold (cm)</label>");
            client.println("<input type=\"text\" id=\"distthreshold\" name=\"distthreshold\" value=\""+ config.lidarDistanceThreshold + "\"><br><br>");
            
            client.println("<label for=\"address\">Smoothing Factor (0-0.9)</label>");
            client.println("<input type=\"text\" id=\"smoothfactor\" name=\"smoothfactor\" value=\""+ config.lidarSmoothingFactor + "\"><br><br>");
            
            client.println("<input type=\"submit\" value=\"Submit\"></form>");
           
            //client.println("<div style=\"text-align: center;\"");
            //client.println("<img src=https://images.squarespace-cdn.com/content/v1/554a673ae4b0d7d5128155bb/1625186042479-70RSPEWTSG8747ZYA8M1/parkdata+logo-01.png?format=150w >");
            // client.println("<img src=http://static1.squarespace.com/static/554a673ae4b0d7d5128155bb/t/5ef63fdf6b62f234d82610a2/1595258253753/?format=150w alt=\"Digame Logo\" >");
            //client.println("</div>");
            client.println("<p style=\"text-align:center; font-style:italic\">Copyright 2021, D&#237game Systems. All rights reserved.</p>");
            
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }  

}



//****************************************************************************************
// Device initialization                                   
//****************************************************************************************
void setup()
{
    initPorts();           // Set up UARTs and GPIOs
    initUI();              // Splash screens 
    
    if (digitalRead(CTR_RESET)== LOW) {
      accessPointMode = true;
      
      debugUART.println("*******************************");
      debugUART.println("Launching in Access Point Mode!");  
      debugUART.println("*******************************");
      
      initFileSystem();      // Setup SD card and load default values.

      debugUART.print("Setting AP (Access Point)…");
      // Remove the password parameter, if you want the AP (Access Point) to be open
      
      WiFi.softAP(ssid);//, password);
    
      IPAddress IP = WiFi.softAPIP();
      debugUART.print("AP IP address: ");
      debugUART.println(IP);
      
      server.begin();
      displayAPScreen(WiFi.softAPIP().toString()); 
      
    } else {
      delay(1000);
      setLowPowerMode();     // Lower clock rate and turn off WiFi    
      initFileSystem();      // Setup SD card and load default values.
      configureLoRa(config); // Configure radio params  
      initLIDAR();           // Turn on LIDAR sensor
      initRealTimeClock();   // Check RTC is present
      
      displayStatusScreen(hwStatus); // Show results of the configuration above
      delay(3000);
  
      mutex_v = xSemaphoreCreateMutex();  //The mutex we will use to protect the jsonMsgBuffer
      xTaskCreate(
        messageManager,      // Function that should be called
        "Message Manager",   // Name of the task (for debugging)
        5000,                // Stack size (bytes)
        NULL,                // Parameter to pass
        1,                   // Task priority
        NULL                 // Task handle
      );

      displayCountScreen(0);
      showValue(0);
    }     
    debugUART.println();
    debugUART.println("RUNNING!");  
    debugUART.println(); 
  
}




//************************************************************************
// Main Loop
//************************************************************************

long count = 0;

void loop()
{ 

  if (accessPointMode){ //***************** Access Point Operation **********************
    
    WiFiClient client = server.available();   // Listen for incoming clients
    if (client){ processClient(client); }     // Handle the web UI

  }else{                //******************* Standard Operation ************************
    //DATA ACQUISITION
    
     vehicleMessageNeeded = processLIDARSignal(); // Threshold Detection
  
     // Check for RESET button being pressed
     if (digitalRead(CTR_RESET)== LOW) {
       count = 0;
       #if SHOW_DATA_STREAM
       #else
          debugUART.print("Loop: RESET button pressed. Count: ");
          debugUART.println(count);  
       #endif
  
       initDisplay();
       displayCountScreen(count);
       showValue(count);
  
     }
  
    //MESSAGE HANDLING
    
      // Runs once at startup.
      if (bootMessageNeeded){
        loraPayload = buildLoRaHeader("b",count);
        loraPayload = loraPayload + "}"; 
        jsonPostNeeded = true;
        bootMessageNeeded = false;
      }
  
      // Issue a heartbeat message every hour.
      heartbeatTime = getRTCMinute();
      if ((oldHeartbeatTime != bootMinute) && (heartbeatTime == bootMinute)){heartbeatMessageNeeded = true;}  
      
      if (heartbeatMessageNeeded){
        loraPayload = buildLoRaHeader("hb",count);
        loraPayload = loraPayload + "}";
        jsonPostNeeded = true;
        heartbeatMessageNeeded = false;
      }
      oldHeartbeatTime = heartbeatTime;
      
      if (vehicleMessageNeeded){ // Set by processLIDARSignal above
        if (buffer.size()==samples){ //Fill up the buffer before processing so we don't get false events at startup.
          
          count++;
          
          #if SHOW_DATA_STREAM
          #else
            debugUART.print("Loop: Count event. Counts: ");
            debugUART.println(count);  
          #endif 
               
          // Consider sliding this into another thread
          if ((count%10==0)&(count>0)) {
            initDisplay();
            displayCountScreen(count);
          }
          showValue(count);
          
          loraPayload = buildLoRaHeader("v",count);
          // Detection algorithm
          String da = "t"; // String(params.detAlgorithm[0]); //[T]hreshold or [C]orrelation
          da.toLowerCase();
          loraPayload = loraPayload + ",\"da\":\"" + da +"\"";
          loraPayload = loraPayload + "}";
        
          jsonPostNeeded = true; 
        }
        vehicleMessageNeeded = false; 
      }
       
      // Push the JSON Payload into the message buffer. The messageManager task will handle it. 
      if (jsonPostNeeded){
        xSemaphoreTake(mutex_v, portMAX_DELAY);  
            loraMsgBuffer.push(loraPayload);
        xSemaphoreGive(mutex_v);
        jsonPostNeeded = false;     
      }  
  }
}
