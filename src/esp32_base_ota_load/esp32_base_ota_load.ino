/*
    Bootstrap OTA compatible firmware loader for ESP32 systems.
    
      * Load onto a bare ESP32 SOM and you have an OTA-enabled SOM, waiting for new code
      * Intended as a production aid
    
    Copyright 2022, Digame Systems. All rights reserved.

*/

//****************************************************************************************
//****************************************************************************************
#include <digameDebug.h>     // Serial debugging defines. 
#include <digameNetwork.h>   // For MAC address functions

//#include <SPIFFS.h>          // FLASH file system support.

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

//****************************************************************************************                            
void setup() // - Device initialization
//****************************************************************************************
{
  Serial.begin(115200);   // Intialize terminal serial port
  delay(1000);            // Give port time to initalize

  DEBUG_PRINTLN("INITIALIZING...");
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
void configureOTA(){
//****************************************************************************************

  DEBUG_PRINTLN("  Stand-Alone Mode. Setting AP (Access Point)…");  
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
  DEBUG_PRINTLN("Digame Bootstrap Load (ESP32 DevKit-C V4");
  DEBUG_PRINTLN("Version 1.0");
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("Compiled: " + compileDate + " at " + compileTime); 
  DEBUG_PRINT("Device Name: ");
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("Copyright 2022, Digame Systems.");
  DEBUG_PRINTLN("All rights reserved.");
  DEBUG_PRINTLN("*******************************************");
  DEBUG_PRINTLN();   
}
