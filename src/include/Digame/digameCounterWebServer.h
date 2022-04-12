#ifndef __DIGAME_COUNTER_WEB_SERVER_H__
#define __DIGAME_COUNTER_WEB_SERVER_H__

/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-web-server-microsd-card/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include <AsyncElegantOTA.h>

#include "FS.h"
#include "SD.h"
#include "SPI.h"

#include <SPIFFS.h>



#include <digameVersion.h> //for SW_VERSION.

#define debugUART Serial
#define tfMiniUART Serial2

#include <digameJSONConfig.h>
#include <digameLIDAR.h>
#include <digameLoRa.h>
#include <digameNetwork.h>
#include <digameTime.h>




// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

bool resetFlag = false;
unsigned long upTimeMillis=0;
const char* http_username = "admin";
const char* http_password = "admin";

unsigned long msLastWebPageEventTime;


//*******************************************************************************************************
String processor(const String& var){
  
  //debugUART.println("Hello from processor");
  //debugUART.println(var);
  
  if(var == "config.lidarZone1Count") return F(String(config.lidarZone1Count).c_str());
  
  if(var == "config.deviceName") return F(config.deviceName.c_str()); 
  if(var == "STREAMING_ON"){if  (config.showDataStream == "true") return F("checked"); } 
  if(var == "STREAMING_OFF"){if (config.showDataStream == "false") return F("checked"); }   
  if(var == "MODEL")             return F(model.c_str());
  if(var == "MODEL_DESCRIPTION") return F(model_description.c_str()); 
  if(var == "SW_VERSION")        return F(SW_VERSION.c_str()); 
  if(var == "MAC_ADDRESS")       return F(getMACAddress().c_str()); 
  
  if(var == "config.heartbeatInterval")  return F(String(config.heartbeatInterval).c_str());
  if(var == "config.ssid")      return F(String(config.ssid).c_str());
  if(var == "config.password")  return F(String(config.password).c_str());
  if(var == "config.serverURL") return F(String(config.serverURL).c_str());

  if(var == "config.loraAddress") return F(String(config.loraAddress).c_str());
  if(var == "config.loraNetworkID") return F(String(config.loraNetworkID).c_str());
  if(var == "config.loraBand") return F(String(config.loraBand).c_str());
  if(var == "config.loraSF") return F(String(config.loraSF).c_str());
  if(var == "config.loraBW") return F(String(config.loraBW).c_str());
  if(var == "config.loraCR") return F(String(config.loraCR).c_str());
  if(var == "config.loraPreamble") return F(String(config.loraPreamble).c_str());
  
  if(var == "config.lidarResidenceTime") return F(String(config.lidarResidenceTime).c_str());
  if(var == "config.lidarZone1Min") return F(String(config.lidarZone1Min).c_str());
  if(var == "config.lidarZone1Max") return F(String(config.lidarZone1Max).c_str());
  if(var == "config.lidarZone2Min") return F(String(config.lidarZone2Min).c_str());
  if(var == "config.lidarZone2Max") return F(String(config.lidarZone2Max).c_str());

  if(var == "config.logBootEvents") return F(String(config.logBootEvents).c_str());
  if(var == "config.logHeartBeatEvents") return F(String(config.logHeartBeatEvents).c_str());  
  if(var == "config.logVehicleEvents") return F(String(config.logVehicleEvents).c_str());
  if(var == "config.logRawData") return F(String(config.logRawData).c_str());
  if(var == "config.counterPopulation") return F(String(config.counterPopulation).c_str());
  if(var == "config.counterID") return F(String(config.counterID).c_str());
  

  if(var == "config.sens1Addr") return F(String(config.sens1Addr).c_str());
  if(var == "config.sens2Addr") return F(String(config.sens2Addr).c_str());
  if(var == "config.sens3Addr") return F(String(config.sens3Addr).c_str());
  if(var == "config.sens4Addr") return F(String(config.sens4Addr).c_str());
  
  if(var == "config.sens1Name") return F(String(config.sens1Name).c_str());
  if(var == "config.sens2Name") return F(String(config.sens2Name).c_str());
  if(var == "config.sens3Name") return F(String(config.sens3Name).c_str());
  if(var == "config.sens4Name") return F(String(config.sens4Name).c_str());

  if(var == "config.sens1MAC") return F(String(config.sens1MAC).c_str());
  if(var == "config.sens2MAC") return F(String(config.sens2MAC).c_str());
  if(var == "config.sens3MAC") return F(String(config.sens3MAC).c_str());
  if(var == "config.sens4MAC") return F(String(config.sens4MAC).c_str());
  
  if(var == "str1Count") return F(String(str1Count).c_str());
  if(var == "str2Count") return F(String(str2Count).c_str());
  if(var == "str3Count") return F(String(str3Count).c_str());
  if(var == "str4Count") return F(String(str4Count).c_str());
  
  return String();  
  
}

//*******************************************************************************************************
void redirectHome(AsyncWebServerRequest* request){
    
    saveConfiguration(filename,config); // Save any changes before redirecting home


    String RedirectUrl = "http://";
    if (ON_STA_FILTER(request)) {
      RedirectUrl += WiFi.localIP().toString();
    } else {
      RedirectUrl += WiFi.softAPIP().toString();
    }
    RedirectUrl += "/";
    request->redirect(RedirectUrl);
}

void processQueryParam(AsyncWebServerRequest *request, String qParam, String *targetParam){
    //debugUART.println(qParam);
    if(request->hasParam(qParam)){
      //debugUART.println("found");
      AsyncWebParameter* p = request->getParam(qParam);

      debugUART.print(p->value());
      debugUART.print(" ");
      debugUART.println(String(p->value()).length());


      if (String(p->value()).length() == 0) {
        //debugUART.println("*******BLANK ENTRY!*******");
        //debgugUART.println("...ignoring...");
      
      } else{
        *targetParam = String(p->value().c_str());
        targetParam->replace("%","_"); // Replace the template character. 
                                     // 'Might cause problems w/ some Passwords...
                                     // TODO: Think on this. Make '%' illegal in PW? 
      }
    }
}



//*******************************************************************************************************
//*******************************************************************************************************
void initWebServer() {

  Serial.println("  Initializing SPIFFS...");


  if(!SPIFFS.begin()){
    Serial.println("    File System Mount Failed");
  } else {
    Serial.println("    SPIFFS up!");
  }

  msLastWebPageEventTime = millis(); // Initialize the web page event timer variable

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    //request->send(SD, "/index.html", String(), false, processor);
    if(!request->authenticate(http_username, http_password))
      return request->requestAuthentication();
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SD, "/params.txt", "text/plain",true);
    redirectHome(request);
  });

  server.on("/eventlog", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SD, "/eventlog.txt", "text/plain",true);
    //redirectHome(request);
  });

  server.on("/cleareventlog", HTTP_GET, [](AsyncWebServerRequest *request){
    //request->send(SD, "/eventlog.txt", "text/plain",true);
    deleteFile("/eventlog.txt");
    redirectHome(request);
  });

  server.on("/histograph", HTTP_GET, [](AsyncWebServerRequest *request){
    debugUART.println("GET /histograph");
    request->send(200, "text/plain", getDistanceHistogramChartString(config));
  });

  server.on("/histo", HTTP_GET, [](AsyncWebServerRequest *request){
    debugUART.println("GET /histo");
    request->send(200, "text/plain", getDistanceHistogramString());
  });
  
  server.on("/counterreset",HTTP_GET, [](AsyncWebServerRequest *request){
    count = 0;
    clearLIDARDistanceHistogram();
    config.lidarZone1Count = "0"; 
    redirectHome(request);
  });

  server.on("/generalparams",HTTP_GET, [](AsyncWebServerRequest *request){
    processQueryParam(request, "devname", &config.deviceName);
    
    String strStream;
    processQueryParam(request, "streaming", &strStream);
    debugUART.println(strStream);
    if (strStream == "ON"){config.showDataStream="true";}else{config.showDataStream="false";}

    config.logBootEvents = "";
    config.logHeartBeatEvents = "";
    config.logVehicleEvents = "";
    config.logRawData = "";

    processQueryParam(request, "logbootevents", &config.logBootEvents);
    processQueryParam(request, "logheartbeatevents", &config.logHeartBeatEvents);
    processQueryParam(request, "logvehicleevents", &config.logVehicleEvents);
    processQueryParam(request, "lograwdata", &config.logRawData);

    String strReboot;
    processQueryParam(request, "reboot", &strReboot);
    debugUART.println(strReboot);
    if (strReboot=="true"){
      resetFlag = true;
    }
    
    redirectHome(request);
     
    
    
  });

  server.on("/networkparams",HTTP_GET, [](AsyncWebServerRequest *request){
    processQueryParam(request, "heartbeatinterval", &config.heartbeatInterval);
    processQueryParam(request, "ssid", &config.ssid);
    processQueryParam(request, "password", &config.password);
    processQueryParam(request, "serverurl", &config.serverURL);  
    redirectHome(request);
  });

  server.on("/loraparams",HTTP_GET, [](AsyncWebServerRequest *request){
    processQueryParam(request, "address", &config.loraAddress);
    processQueryParam(request, "networkid", &config.loraNetworkID);
    processQueryParam(request, "band", &config.loraBand);
    processQueryParam(request, "spreadingfactor", &config.loraSF);
    processQueryParam(request, "bandwidth", &config.loraBW);
    processQueryParam(request, "codingrate", &config.loraCR);
    processQueryParam(request, "preamble", &config.loraPreamble); 

    //initLoRa();
    //configureLoRa(config);
    redirectHome(request);
  });

  server.on("/lidarparams",HTTP_GET, [](AsyncWebServerRequest *request){

    processQueryParam(request, "counterid", &config.counterID);
    processQueryParam(request, "counterpopulation", &config.counterPopulation);
    processQueryParam(request, "residencetime", &config.lidarResidenceTime);
    processQueryParam(request, "zone1min", &config.lidarZone1Min);
    processQueryParam(request, "zone1max", &config.lidarZone1Max);
    processQueryParam(request, "zone2min", &config.lidarZone2Min);
    processQueryParam(request, "zone2max", &config.lidarZone2Max);
    redirectHome(request);
  });


  server.on("/sensors",HTTP_GET, [](AsyncWebServerRequest *request){
    processQueryParam(request, "sens1name", &config.sens1Name);
    processQueryParam(request, "sens1addr", &config.sens1Addr);
    processQueryParam(request, "sens1mac",  &config.sens1MAC);
    
    processQueryParam(request, "sens2name", &config.sens2Name);
    processQueryParam(request, "sens2addr", &config.sens2Addr);
    processQueryParam(request, "sens2mac",  &config.sens2MAC);
    
    processQueryParam(request, "sens3name", &config.sens3Name);
    processQueryParam(request, "sens3addr", &config.sens3Addr);
    processQueryParam(request, "sens3mac",  &config.sens3MAC);
    
    processQueryParam(request, "sens4name", &config.sens4Name);
    processQueryParam(request, "sens4addr", &config.sens4Addr);
    processQueryParam(request, "sens4mac",  &config.sens4MAC);
    
    redirectHome(request);
  });

  server.on("/distance", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", lastDistanceMeasured+","+\
                                     String(config.lidarZone1Count));
    msLastWebPageEventTime = millis();
  });

  server.on("/counters", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", str1Count+","+\
                                     str2Count+","+\
                                     str3Count+","+\
                                     str4Count); 
  });

  server.on("/uptime", HTTP_GET, [](AsyncWebServerRequest *request){
    String s = TimeToString(upTimeMillis/1000);
    //s = TimeToString(312847); // Testing. = (3 days 14 hours 54 min 7 seconds)
    //debugUART.print("/uptime: ");
    //debugUART.println(s);
    request->send(200, "text/plain", s);  // Report in seconds
  });


  //server.serveStatic("/", SD, "/");
  server.serveStatic("/", SPIFFS, "/");
 
  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
}

#endif