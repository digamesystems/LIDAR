
#ifndef __DIGAME_NETWORK_H__
#define __DIGAME_NETWORK_H__

//#include <WiFi.h>       // WiFi stack
#define debugUART Serial

//****************************************************************************************
// Return the device's MAC address
String getMACAddress(){
  byte mac[6];
  String retString;

#if USE_WIFI
  WiFi.macAddress(mac);
  retString = String(String(mac[5],HEX)+":");
  retString = String(retString + String(mac[4],HEX) +":");
  retString = String(retString + String(mac[3],HEX) +":");  
  retString = String(retString + String(mac[2],HEX) +":");
  retString = String(retString + String(mac[1],HEX) +":");
  retString = String(retString + String(mac[0],HEX));
#else 
  retString = "00:01:02:03:04:05";
#endif
  return retString;
}

//****************************************************************************************
// Attempt to connect to the WiFi Network.
bool connectToWiFi(String stringSSID, String stringPassword){
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_OFF); // Let's start fresh...
  delay(1000);
  WiFi.begin(stringSSID.c_str(), stringPassword.c_str());
  long t1 = millis();
  bool timeOut = false;

  while ((WiFi.status() != WL_CONNECTED) && (millis() - t1 < 5000) ) {
    delay(500);
    debugUART.print(".");
  }
  if (millis()-t1>=5000){
    debugUART.print(" TIMEOUT!");
    return false;  
  } else{
    debugUART.print(" CONNECTED!");
    return true;
  }
}


//****************************************************************************************
// Attempt to make a WiFi connection.
bool initWiFi(String stringSSID, String stringPassword){
  
  int maxRetries = 3;
  int retries = 0;

  while (retries < maxRetries){
    if (connectToWiFi(stringSSID, stringPassword)) {return true;};
    retries++;
  }
  return false; 
}


#endif //__DIGAME_NETWORK_H__
