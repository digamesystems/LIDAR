/*
    Bootstrap OTA compatible firmware loader for ESP32 systems.
    
      * Load onto a bare ESP32 SOM and you have an OTA-enabled SOM, waiting for new code
      * Intended as a production aid
    
    Copyright 2022, Digame Systems. All rights reserved.

*/

//****************************************************************************************
//****************************************************************************************
#include <digameDebug.h>        // Serial debugging defines. 
#include <digameTime.h>         // Digame Time Functions
#include <digameNetwork_v2.h>   // Networking, MAC address, etc. 
#include <digameDisplay.h>

// For Over the Air (OTA) updates... 
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

bool useOTA = true; 

//****************************************************************************************
//****************************************************************************************                        
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

void showSplashScreen();
void configureOTA();
void configureRTC();
void configureDisplay();

//****************************************************************************************                            
void setup() // - Device initialization
//****************************************************************************************
{
  Serial.begin(115200);   // Intialize terminal serial port
  delay(1000);            // Give port time to initalize

  showSplashScreen();
  configureDisplay();
  configureRTC();
  configureOTA();  
 
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("RUNNING!");
     
}


//****************************************************************************************
void loop()  // Main 
//****************************************************************************************
{ 
 delay(10); // Nothing to do except wait for a firmware load.
}

//****************************************************************************************
void configureDisplay(){
//****************************************************************************************
  initDisplay();
  displayTextScreen("Screen Test", "OK?");
}


//****************************************************************************************
void configureRTC(){
//****************************************************************************************
  NetworkConfig config;
  bool updateRTC = false;
  config.ssid       = (const char *)"Bighead";   // YOUR_NETWORK_NAME 
  config.password   = (const char *)"billgates"; // YOUR_PASSWORD

  Wire.begin();
  
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("  Testing for Real-Time-Clock module...");
  
  if (rtcPresent()){
    DEBUG_PRINT("    RTC found. Time: ");
    DEBUG_PRINTLN(getRTCTime()); 

    DEBUG_PRINT(  "    Update? y/[n] (You have 5 sec to decide) ");
    unsigned long t1 = millis();
    unsigned long t2 = t1;

    while (!(debugUART.available()) && ((t2-t1)<5000)) {
      t2 = millis();
      delay(500); // wait for data from the user... 
      DEBUG_PRINT(".");
    }

    DEBUG_PRINTLN();

    if (debugUART.available()){
      String ynString = debugUART.readStringUntil('\n');
      ynString.trim();
      if (ynString == "y") {updateRTC = true;}
    }
  
    if (updateRTC){
      enableWiFi(config);
      DEBUG_PRINTLN("    MAC Address: " + getMACAddress());
    } 
     
  }else{
    DEBUG_PRINTLN("    ERROR! Could NOT find RTC.");   
  }

  if (wifiConnected){ // Attempt to synch ESP32 and DS3231 with NTP server
    synchTimesToNTP();
  }
    
  DEBUG_PRINTLN();

}



//****************************************************************************************
void configureOTA(){
//****************************************************************************************

  DEBUG_PRINTLN("  Starting Over the Air Update Mode. Setting AP (Access Point)…");  
  WiFi.mode(WIFI_AP);
  
  String netName = "Uninitialized_" + getShortMACAddress();
  const char* ssid = netName.c_str();
  WiFi.softAP(ssid);
  
  IPAddress IP = WiFi.softAPIP();
  DEBUG_PRINT("    AP IP address: ");
  DEBUG_PRINTLN(IP);  
  DEBUG_PRINTLN("    Network Name: " + netName); 
  
  //delay(3000);  
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    //if(!request->authenticate("admin", "admin"))
    
    String RedirectUrl = "http://";
    if (ON_STA_FILTER(request)) {
      RedirectUrl += WiFi.localIP().toString();
    } else {
      RedirectUrl += WiFi.softAPIP().toString();
    }
    
    RedirectUrl += "/update";
    request->redirect(RedirectUrl);
    
  });

  AsyncElegantOTA.begin(&server);   
  server.begin();
}
  
//****************************************************************************************
void showSplashScreen(){
//****************************************************************************************
  String compileDate = F(__DATE__);
  String compileTime = F(__TIME__);

  DEBUG_PRINTLN();
  DEBUG_PRINTLN("*******************************************");
  DEBUG_PRINTLN("Digame Bootstrap Load (ESP32 DevKit-C V4)");
  DEBUG_PRINTLN("Version 1.0");
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("Compiled: " + compileDate + " at " + compileTime); 
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("Copyright 2022, Digame Systems.");
  DEBUG_PRINTLN("All rights reserved.");
  DEBUG_PRINTLN("*******************************************");
  DEBUG_PRINTLN();   
}
