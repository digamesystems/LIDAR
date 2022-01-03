/* digameNetwork.h
 *  
 *  Functions for logging into a wifi network and reporting the device's
 *  MAC address, and POSTING a JSON message to a server.
 *  
 *  Copyright 2021, Digame Systems. All rights reserved.  
 */

#ifndef __DIGAME_OTA_H__
#define __DIGAME_OTA_H__

#include <WiFi.h>             // WiFi stack
#include <digameJSONConfig.h> // for Config struct that holds network credentials
#include <ArduinoOTA.h>

#define debugUART Serial


//****************************************************************************************
// Configure for Over the Air Programming (OTA)
void initOTA(){
  
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);
  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname(String(String("Digame-") + getShortMACAddress()).c_str());

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
      
}



#endif //__DIGAME_OTA_H__
