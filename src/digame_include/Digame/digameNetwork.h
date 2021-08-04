/* digameNetwork.h
 *  
 *  Functions for logging into a wifi network and reporting the device's
 *  MAC address.
 *  
 *  Copyright 2021, Digame Systems. All rights reserved.  
 */

#ifndef __DIGAME_NETWORK_H__
#define __DIGAME_NETWORK_H__

#include <WiFi.h> // WiFi stack

#define debugUART Serial

// Globals
bool wifiConnected = false; 

//************************************************************************
// Return the device's MAC address as a String
String getMACAddress(){
  byte mac[6];
  String retString;

  WiFi.macAddress(mac);
  retString = String(String(mac[5],HEX)+":");
  retString = String(retString + String(mac[4],HEX) +":");
  retString = String(retString + String(mac[3],HEX) +":");  
  retString = String(retString + String(mac[2],HEX) +":");
  retString = String(retString + String(mac[1],HEX) +":");
  retString = String(retString + String(mac[0],HEX));

  return retString;
}

//************************************************************************
// Enable WiFi and log into the network
void enableWiFi(String ssid, String password){
 
    WiFi.disconnect(false);  // Reconnect the network
    WiFi.mode(WIFI_OFF);     // Switch WiFi off
 
    delay(200);
 
    debugUART.print("Starting WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
 
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
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF);    // Switch WiFi off
    debugUART.println("");
    debugUART.println("WiFi disconnected!");
}



#endif //__DIGAME_NETWORK_H__
