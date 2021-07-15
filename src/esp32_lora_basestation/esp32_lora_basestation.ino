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

#include <ArduinoJson.h>

#define loraUART Serial1
#define debugUART Serial

// Set web server port number to 80
WiFiServer server(80);

Config config;

int RX_LED    = 13;
int LED_DIAG  = 12;  // Indicator LED
int CTR_RESET = 32;  // Counter Reset Input

long   msLastConnectionAttempt = 0;
bool   wifiConnected = false;
bool   rtcPresent    = false;

int    displayMode = 1; // 1=Title, 2=Network, 3=Last Message

bool   loraConfigMode = false;
String cmdMsg;
String loraMsg;

const char *ssid = "Digame-AP";
const char *password ="digame";

SemaphoreHandle_t mutex_v; // Mutex used to protect our jsonMsgBuffers (see below).

const int samples = 20;
CircularBuffer<String, samples> loraMsgBuffer; // A buffer containing JSON messages sent to the LoRa basestation.

bool accessPointMode = false;

String sendReceive(String s);
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

  // first update should be full refresh
  initDisplay();
  displaySplashScreen();

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

  static String lastMessageTimeStamp;

  StaticJsonDocument<512> doc;

  debugUART.println("processing: ");
  debugUART.println(msg);
  // Get the device's Address.
  int idxstart = msg.indexOf('=')+1;
  int idxstop  = msg.indexOf(','); 
  String strAddress = msg.substring(idxstart,idxstop);
  debugUART.println(strAddress);
  
  // Start and end of the JSON payload in the msg.
  idxstart = msg.indexOf('{');
  idxstop = msg.indexOf('}')+1;

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
  strTime = "20" + strTime;
   
  // Count
  String strCount = doc["c"];
  if (lastMessageTimeStamp.equals(strCount)) {
    debugUART.println("We've seen this message before.");
    return;  
  }
  lastMessageTimeStamp = strCount;
 
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

  // TODO: move to a look up function and come up with a better storage scheme.
  if (strAddress == config.sens1Addr){
    strDeviceName = config.sens1Name;
    strDeviceMAC = config.sens1MAC;    
  } else if (strAddress == config.sens2Addr){
    strDeviceName = config.sens2Name;
    strDeviceMAC = config.sens2MAC;
  } else if (strAddress == config.sens3Addr){
    strDeviceName = config.sens3Name;
    strDeviceMAC = config.sens3MAC;
  } else if (strAddress == config.sens4Addr){
    strDeviceName = config.sens4Name;
    strDeviceMAC = config.sens4MAC;
  }
  
  if (displayMode ==3) { // Update the eInk display with the latest information
    String strDisplay="";
    strDisplay =   " Addr:  " + strAddress + 
                 "\n Event: " + strEventType + 
                 "\n Count: " + strCount + 
                 "\n Temp:  " + strTemperature +
                 "\n Date:  " + strTime.substring(0,strTime.indexOf(" ")) +
                 "\n Time:  " + strTime.substring(strTime.indexOf(" ")+1); 
    displayStatusScreen(strDisplay);
  
  }


  jsonPayload = "{\"deviceName\":\""       + strDeviceName + 
                 "\",\"deviceMAC\":\""     + strDeviceMAC  + 
                 "\",\"timeStamp\":\""     + strTime + 
                 "\",\"linkMode\":\""      + "LoRa" +
                 "\",\"eventType\":\""     + strEventType + 
                 "\",\"detAlgorithm\":\""  + strDetAlg +
                 "\",\"count\":\""         + strCount + 
                 "\",\"rssi\":\""          + strRSSI + 
                 "\",\"snr\":\""           + strSNR +    
                 "\",\"temp\":\""          + strTemperature +                
                 "\"}";

  debugUART.println(jsonPayload);
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
    debugUART.println("Bluetooth stopped!");
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
String sendReceive(String s){
  //String strReturn;
  String loraMsg;

  //Send the command
  loraUART.println(s);

  //Read reply
  while (!loraUART.available()) {
    delay(10);
  }
  loraMsg = loraUART.readStringUntil('\n');
  //}

  //debugUART.println(loraMsg);
  return loraMsg;
  
}

//************************************************************************
void configureLoRa(Config config){
  
  debugUART.println("Configuring LoRa...");
  
  debugUART.println("  Setting Address to: " + config.loraAddress);
  sendReceive("AT+ADDRESS="+config.loraAddress);
  sendReceive("AT+ADDRESS?");
  sendReceive("AT+ADDRESS?");

  debugUART.println("  Setting Network ID to: " + config.loraNetworkID);
  sendReceive("AT+NETWORKID="+config.loraNetworkID);
  sendReceive("AT+NETWORKID?");
  
  debugUART.println("  Setting Band to: " + config.loraBand);
  sendReceive("AT+BAND="+config.loraBand);
  sendReceive("AT+BAND?");

  debugUART.println("  Setting Modulation Parameters to: " + config.loraSF+","+config.loraBW+","+config.loraCR+","+config.loraPreamble);
  sendReceive("AT+PARAMETER="+config.loraSF+","+config.loraBW+","+config.loraCR+","+config.loraPreamble);
  sendReceive("AT+PARAMETER?");
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
//****************************************************************************************
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
              
            } else if (header.indexOf("GET /network") >=0) {
              Serial.println("You are tweaking network parameters.");  
              config.ssid = getParam(header,"ssid");
              config.password  = getParam(header,"password");
              config.serverURL = getParam(header,"serverurl");
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
              configureLoRa(config);
            
            } else if (header.indexOf("GET /sensors") >=0){
              Serial.println("You are tweaking Sensor parameters.");
              config.sens1Name  = getParam(header,"sens1name");
              config.sens1Addr  = getParam(header,"sens1addr");
              config.sens1MAC   = getParam(header,"sens1mac");             
              config.sens2Name  = getParam(header,"sens2name");
              config.sens2Addr  = getParam(header,"sens2addr");
              config.sens2MAC   = getParam(header,"sens2mac"); 
              config.sens3Name  = getParam(header,"sens3name");
              config.sens3Addr  = getParam(header,"sens3addr");
              config.sens3MAC   = getParam(header,"sens3mac"); 
              config.sens4Name  = getParam(header,"sens4name");
              config.sens4Addr  = getParam(header,"sens4addr");
              config.sens4MAC   = getParam(header,"sens4mac"); 
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

            client.println("<br><H2>BASE STATION <br><em>"+ config.deviceName +"</em></H2>");
            client.println("<p>Welcome. You can use these forms to update system parameters for this LoRa-WiFi base station.</p>");

            client.println("<form action=\"/generalparms\">\n");
            client.println("<H3>General</H3>");
            
            client.println("<label for=\"devname\">Device Name</label>");
            client.println("<input type=\"text\" id=\"devname\" name=\"devname\" value=\""+ config.deviceName + "\"><br><br>");
            client.println("<label>Model<br>");
            client.println("DS-VC-BASE-LOR-1<br></label><p><small>(Single-Channel, LoRa-WiFi Gateway)</small></p>");
            
            client.println("<input type=\"submit\" value=\"Submit\"></form>");

            client.println("<form action=\"/networkparms\">\n");
            client.println("<H3>Network</H3>");
            client.println("<p>NOTE: Changes to network settings will be applied after a reboot of the base station. </p>");
            
            client.println("<label>MAC Address: " + getMACAddress() + "</label><br><br>");
            
            client.println("<label for=\"ssid\">SSID</label>");
            client.println("<input type=\"text\" id=\"ssid\" name=\"ssid\" value=\""+ config.ssid + "\"><br><br>\n");
            
            client.println("<label for=\"password\">Password</label>");
            client.println("<input type=\"text\" id=\"password\" name=\"password\" value=\""+ config.password +"\"><br><br>");
            
            client.println("<label for=\"serverurl\">Server URL</label>");
            client.println("<input type=\"text\" id=\"serverurl\" name=\"serverurl\" value=\""+ config.serverURL + "\"</input><br><br>");
            client.println("<input type=\"submit\" value=\"Submit\"></form>");

            client.println("<form action=\"/loraparms\">\n");
            client.println("<H3>LoRa (Low-Power Long-Range Wireless Link)</H3>");
            client.println("<p>Strategies for optimizing the parameters for LoRa communication may be found in the product manual. <TODO: LINK> Both the counters and the base station must use the same RF parameters for communication.</p>");
            client.println("<label for=\"address\">Base Station Address</label>");
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

            client.println("<form action=\"/sensors\">\n");
            client.println("<H3>Sensors</H3>");

            client.println("<p>A base station can support up to four (4) traffic counters. Each counter needs a unique LoRa address to communicate. The MAC address for the counter can be found on the label on the back of the sensor.</p>");
            
            client.println("<table>");      
            client.println("<thead><th>Name</th><th>Addr</th><th>Mac Address</th></thead>");
            client.println("<tbody>");
            
            client.println("<tr>");
            client.println("<td><input type=\"text\" id=\"sens1name\" name=\"sens1name\" value=\"" + config.sens1Name + "\"></td>");
            client.println("<td><input class=\"narrow\" type=\"text\" id=\"sens1addr\" name=\"sens1addr\" value=\""+ config.sens1Addr + "\"></td>");
            client.println("<td><input type=\"text\" id=\"sens1mac\" name=\"sens1mac\" value=\"" + config.sens1MAC + "\"></td>");
            client.println("</tr>");

            client.println("<tr>");
            client.println("<td><input type=\"text\" id=\"sens2name\" name=\"sens2name\" value=\"" + config.sens2Name + "\"></td>");
            client.println("<td><input class=\"narrow\" type=\"text\" id=\"sens2addr\" name=\"sens2addr\" value=\""+ config.sens2Addr + "\"></td>");
            client.println("<td><input type=\"text\" id=\"sens2mac\" name=\"sens2mac\" value=\"" + config.sens2MAC + "\"></td>");
            client.println("</tr>");

            client.println("<tr>");
            client.println("<td><input type=\"text\" id=\"sens3name\" name=\"sens3name\" value=\"" + config.sens3Name + "\"></td>");
            client.println("<td><input class=\"narrow\" type=\"text\" id=\"sens3addr\" name=\"sens3addr\" value=\""+ config.sens3Addr + "\"></td>");
            client.println("<td><input type=\"text\" id=\"sens3mac\" name=\"sens3mac\" value=\"" + config.sens3MAC + "\"></td>");
            client.println("</tr>");

            client.println("<tr>");
            client.println("<td><input type=\"text\" id=\"sens4name\" name=\"sens4name\" value=\"" + config.sens4Name + "\"></td>");
            client.println("<td><input class=\"narrow\" type=\"text\" id=\"sens4addr\" name=\"sens4addr\" value=\""+ config.sens4Addr + "\"></td>");
            client.println("<td><input type=\"text\" id=\"sens4mac\" name=\"sens4mac\" value=\"" + config.sens4MAC + "\"></td>");
            client.println("</tr>");          

            client.println("</tbody>");
         
            client.println("</table><br>");
            client.println("<input type=\"submit\" value=\"Submit\"></form>");

            
            //client.println("<div style=\"text-align: center;\"");
            //client.println("<img src=https://images.squarespace-cdn.com/content/v1/554a673ae4b0d7d5128155bb/1625186042479-70RSPEWTSG8747ZYA8M1/parkdata+logo-01.png?format=150w >");
             client.println("<img src=http://static1.squarespace.com/static/554a673ae4b0d7d5128155bb/t/5ef63fdf6b62f234d82610a2/1595258253753/?format=150w alt=\"Digame Logo\" >");
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



//************************************************************************
// Setup
//************************************************************************
void setup() {
  initPorts(); //Set up serial ports and GPIO
  splash();
    
  debugUART.println("INITIALIZING\n");

  if (digitalRead(CTR_RESET)== LOW) {
    debugUART.println("*******************************");
    debugUART.println("Launching in Access Point Mode!");  
    debugUART.println("*******************************");
    
    accessPointMode = true;
  }

  debugUART.println("Reading Parameters from SD Card...");
  initSDCard();

  //saveConfiguration(filename,config);
  loadConfiguration(filename,config);

  if (accessPointMode){
      Serial.print("Setting AP (Access Point)…");
      // Remove the password parameter, if you want the AP (Access Point) to be open
      
      WiFi.softAP(ssid);//, password);
    
      IPAddress IP = WiFi.softAPIP();
      Serial.print("AP IP address: ");
      Serial.println(IP);
      
      server.begin();
      displayAPScreen(WiFi.softAPIP().toString()); 
         
      
    
    
  }else{

    //config.ssid = "Bighead";
    //config.password = "billgates";
    debugUART.println("  Device Name: " + config.deviceName);
    debugUART.println("  SSID: " + config.ssid);
    debugUART.println("  ServerURL: " + config.serverURL);
    
    //printFile(filename);  
    
    setNormalMode(); //Run at full power and max speed with WiFi enabled by default
    debugUART.println("  MAC Address: " + getMACAddress());  
  
    configureLoRa(config);
  
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
      //displayIPScreen(WiFi.localIP().toString());
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
    
  }
  
  server.begin();
    
  debugUART.println();
  debugUART.println("RUNNING\n");
}


//************************************************************************
// Main Loop
//************************************************************************
void loop() {

  WiFiClient client = server.available();   // Listen for incoming clients

  if (client){ processClient(client); }


  if (!accessPointMode){
 
    // Check for RESET button being pressed
    if (digitalRead(CTR_RESET)== LOW) {
      //debugUART.println("Loop: RESET button pressed.");
      displayMode++;
      if (displayMode>3) displayMode = 1;
      
      switch (displayMode) {
        case 1:
          displaySplashScreen();
          break;
        case 2:
          displayIPScreen(WiFi.localIP().toString()); 
          break;
        case 3:
          displayStatusScreen("");
          break;     
      }
    }
  
    // Handle what the LoRa module has to say. 
    // If it's a message from another module, add it to the queue
    // so the manager function can handle it.
    // Otherwise, just echo to the debugUART.
    
    if (loraUART.available()) {
      
      loraMsg = loraUART.readStringUntil('\n');
         
      //Messages received by the Module start with '+RCV'
      if (loraMsg.indexOf("+RCV")>=0){
        debugUART.println();
        debugUART.print("LoRa Message Received. ");  
        debugUART.println(loraMsg);
        
        // Send an acknowlegement to the sender.
        // Grab the address of the sender
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
}
