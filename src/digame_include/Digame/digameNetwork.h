/* digameNetwork.h
 *  
 *  Functions for logging into a wifi network and reporting the device's
 *  MAC address, and POSTING a JSON message to a server.
 *  
 *  Copyright 2021, Digame Systems. All rights reserved.  
 */

#ifndef __DIGAME_NETWORK_H__
#define __DIGAME_NETWORK_H__

#include <WiFi.h>             // WiFi stack
#include <HTTPClient.h>       // To post to the ParkData Server
#include <digameJSONConfig.h> // for Config struct that holds network credentials

#define debugUART Serial

// Globals
bool wifiConnected = false;
long msLastConnectionAttempt; // Timer value of the last time we tried to connect to the wifi.
HTTPClient http;  // The class we use to POST messages


//*****************************************************************************
// Return the device's MAC address as a String
String getMACAddress()
{

    byte mac[6];
    String retString;

    WiFi.macAddress(mac);
    retString = String(String(mac[0], HEX) + ":");
    retString = String(retString + String(mac[1], HEX) + ":");
    retString = String(retString + String(mac[2], HEX) + ":");
    retString = String(retString + String(mac[3], HEX) + ":");
    retString = String(retString + String(mac[4], HEX) + ":");
    retString = String(retString + String(mac[5], HEX));

    return retString;
}

//*****************************************************************************
// Return the last four digits of the device's MAC address as a String
String getShortMACAddress()
{

    byte mac[6];
    String retString;

    WiFi.macAddress(mac);
    retString = String(retString + String(mac[4], HEX));
    retString = String(retString + String(mac[5], HEX));

    return retString;
}
//*****************************************************************************
// Enable WiFi and log into the network
bool enableWiFi(Config config)
{

    String ssid = config.ssid;
    String password = config.password;

    WiFi.disconnect();   // Disconnect the network
    WiFi.mode(WIFI_OFF); // Switch WiFi off

    delay(1000); // Wait a bit...

    debugUART.print("  Starting WiFi");
    WiFi.mode(WIFI_STA); // Station mode
    delay(250);
    WiFi.begin(ssid.c_str(), password.c_str()); // Log in


    bool timedout = false;
    unsigned long wifiTimeout = 10000;
    unsigned long tstart = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        debugUART.print(".");
        if ((millis() - tstart) > wifiTimeout)
        {
            debugUART.println("Timeout attempting WiFi connection.");
            timedout = true;
            break;
        }
    }

    msLastConnectionAttempt = millis(); // Timer value of the last time we tried to connect to the wifi.

    if (timedout)
    {
        wifiConnected = false;
        return false;
    }
    else
    {
        debugUART.println("");
        debugUART.println("    WiFi connected.");
        debugUART.print("    IP address: ");
        debugUART.println(WiFi.localIP());
        wifiConnected = true;

        // Your Domain name with URL path or IP address with path
        // http.begin(config.serverURL);

        return true;
    }
}

//*****************************************************************************
void disableWiFi()
{
    WiFi.disconnect(true); // Disconnect from the network
    WiFi.mode(WIFI_OFF);   // Switch WiFi off
    debugUART.println("");
    debugUART.println("WiFi disconnected!");
    wifiConnected = false;
}

//*****************************************************************************
// Save a single JSON message to the server. TODO: Deal with retries, etc.
bool postJSON(String jsonPayload, Config config)
{

    if (showDataStream == false){
        debugUART.print("postJSON Running on Core #: ");
        debugUART.println(xPortGetCoreID());
        //debugUART.print("Free Heap: ");
        //debugUART.println(ESP.getFreeHeap());
    }

    if (WiFi.status() != WL_CONNECTED)
    {
        debugUART.println("WiFi Connection Lost.");
        if (enableWiFi(config) == false)
        {
            return false;
        };
    }

    unsigned long t1 = millis();

    // Your Domain name with URL path or IP address with path
    http.begin(config.serverURL);
    //http.begin("http://199.21.201.53/trailwaze/zion/lidar_sensor_import.php");

    if (showDataStream == false){
        debugUART.print("JSON payload length: ");
        debugUART.println(jsonPayload.length());
        debugUART.print("HTTP begin Time: ");
        debugUART.println(millis() - t1);
    }
    
    // If you need an HTTP request with a content type: application/json, use the following:
    http.addHeader("Content-Type", "application/json");

    t1 = millis();
    int httpResponseCode = http.POST(jsonPayload);

if (showDataStream == false){
    debugUART.print("POST Time: ");
    debugUART.println(millis() - t1);
    debugUART.println("POSTing to Server:");
    debugUART.println(jsonPayload);
    debugUART.print("HTTP response code: ");
    if (!(httpResponseCode==200)){
        debugUART.println("*****ERROR*****");
    }
    debugUART.println(httpResponseCode);
    debugUART.println();
}

    // Free resources
    http.end();

    if (httpResponseCode==200){
      return true;
    } else { 
      return false;
    }
    
}

#endif //__DIGAME_NETWORK_H__
