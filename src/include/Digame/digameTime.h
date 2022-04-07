/* digameTime.h
 *  
 *  Functions for working with an external, battery-backed RTC. 
 *  
 *  The ESP32 has an internal RTC that can be set from an NTP
 *  server. However, this RTC is not battery backed, so power 
 *  interruptions reset it. 
 *  
 *  This library allows you to set the RTC on an external, 
 *  battery-backed RTC from an NTP server. 
 *  
 *  We are using a DSP3231 module in our application:   
 *  Adafruit DS3231 RTC Module: https://www.adafruit.com/product/3013
 *  
 *  Copyright 2021, Digame Systems. All rights reserved.  
 */
 
#ifndef __DIGAME_TIME_H__
#define __DIGAME_TIME_H__

#define debugUART Serial

#include <Wire.h> 
#include "time.h"   // UTC functions
#include <DS3231.h> // Real Time Clock Library


// Globals
const char* ntpServer          = "time.nist.gov"; //"pool.ntp.org"; // Time server.
const long  gmtOffset_sec      = 0; // No timezone offset (using GMT)
const int   daylightOffset_sec = 0; // No Daylight Savings time offset 

// Declares
bool   initRTC(); 
String getESPTime();   // Returns GMT time in the ESP32's internal RTC

bool   rtcPresent();   // Tests for presence of the DS3231 module
String getRTCTime();   // Returns GMT time in the DS3231 module
bool   setRTCTime();   // Set the DS3231 module to the same time as the 
                       // ESP's internal RTC
int    getRTCSecond(); // The current second
int    getRTCMinute(); // The current minute
int    getRTCHour();   // The current hour
float  getRTCTemperature();

bool   synchTimesToNTP(); // Get time from an NTP server and set both
                          // clocks

String twoDigits(byte); // Utility function to format integers as two-
                        // digit strings


// t is time in seconds = millis()/1000;
String TimeToString(unsigned long t)
{
  static char str[12];
  
  long d = (t / 3600) / 24;
  
  long h = (t / 3600) - (d * 24);

  t = t % 3600;
  
  int m = t / 60;
  int s = t % 60;

  sprintf(str, "%03ld:%02d:%02d:%02d", d, h, m, s);
  //debugUART.print(" Uptime: ");
  String ret = String(str);
  //debugUART.println(ret);
  return ret;
}

//****************************************************************************************
bool initRTC(){
  Wire.begin();
  debugUART.println("  Testing for Real-Time-Clock module... ");
  if (rtcPresent()){
    debugUART.println("    RTC found. (Program will use time from RTC.)");
    //stat +="   RTC  : OK\n";
    debugUART.print("    RTC Time: ");
    debugUART.println(getRTCTime()); 
    return true;
  }else{
    debugUART.println("    ERROR! Could NOT find RTC. (Program will attempt to use NTP time.)");   
    //stat +="    RTC  : ERROR!\n";
    return false; 
  }  

}

//*****************************************************************************
// "ESP Time" is the time in the ESP32's internal RTC. -- This isn't 
// battery backed, but can be set using the NTP. 
String getESPTime()
{
  String retStr;
  struct tm timeinfo;
  
  if(!getLocalTime(&timeinfo)){
    retStr = String("Failed to obtain time");
    return retStr;
  }
  
  // Format:
  // YYYY-MM-DD HH:MM:SS
  // debugUART.print(&timeinfo, "%Y-%m-%d %H:%M:%S");
  retStr = String(timeinfo.tm_year + 1900);
  retStr = retStr + ("-");
  retStr = retStr + twoDigits(timeinfo.tm_mon+1);
  retStr = retStr + ("-");
  retStr = retStr + twoDigits(timeinfo.tm_mday);
  retStr = retStr + (" ");
  retStr = retStr + twoDigits(timeinfo.tm_hour);
  retStr = retStr + (":");
  retStr = retStr + twoDigits(timeinfo.tm_min);
  retStr = retStr + (":");
  retStr = retStr + twoDigits(timeinfo.tm_sec);
  return retStr;
}

//*****************************************************************************
bool rtcPresent(){
  String tNow;

  tNow = getRTCTime();
  
  //debugUART.println(tNow);
  
  // When there is no RTC module connected, getRTCTime returns a string 
  // with the wrong format to be a real time value. A simple check is
  // the length of the string. A better test would be to use a cute 
  // regular expression. TODO: Look into that.

  // Format: 2021-01-01 00:00:00 (length = 19) 
  return (tNow.length() == 19); 
  
}

//*****************************************************************************
// Retrieve the time from the external, battery-backed RTC and format. 
String getRTCTime(){
  
  DS3231 clock;
  String message = "20";

  message += twoDigits(clock.getYear());
  message += "-";
  bool cent;
  message += twoDigits(clock.getMonth(cent));
  message += "-";
  message += twoDigits(clock.getDate());
  message +=" ";
  bool hr;
  bool amPM;
  message += twoDigits(clock.getHour(hr,amPM));
  message += ':';
  message += twoDigits(clock.getMinute());
  message += ':';
  message += twoDigits(clock.getSecond());  

  //debugUART.println(message);

  if (message.length()!=19) {
    message = "No RTC found.";  
  }

  //Serial.println(clock.getTemperature());
  return message;   
}

//*****************************************************************************
int getRTCSecond(){
  RTClib myRTC;
  DateTime now = myRTC.now();
  return now.second(); 
}

//*****************************************************************************
int getRTCMinute(){
  RTClib myRTC;
  DateTime now = myRTC.now();
  return now.minute(); 
}

//*****************************************************************************
int getRTCHour(){
  RTClib myRTC;
  DateTime now = myRTC.now();
  return now.hour(); 
}

//*****************************************************************************
float getRTCTemperature(){
    DS3231 clock;
    return clock.getTemperature(); 
}


//*****************************************************************************
// Attempt to synchronize the external RTC with the ESP32's internal RTC.
// If the internal RTC isn't set, leave the current value in the RTC.
bool setRTCTime(){
  
  String retStr;
  struct tm timeinfo;
  
  if(!getLocalTime(&timeinfo)){ // Read from the ESP32 RTC (not battery backed...) 
    retStr = String("Failed to obtain NTP time");
    return false;
  }

  // Set the external RTC
  DS3231 clock;
  clock.setClockMode(false); // 24 hour time
  clock.setYear(timeinfo.tm_year-100);
  clock.setMonth(timeinfo.tm_mon+1);
  clock.setDate(timeinfo.tm_mday);

  clock.setHour(timeinfo.tm_hour);
  clock.setMinute(timeinfo.tm_min);
  clock.setSecond(timeinfo.tm_sec);
  
  return true;
}  

//*****************************************************************************
// Get the time from an NTP server and copy it into the external RTC
bool synchTimesToNTP(){
  
  debugUART.println("  Getting time from NTP Server... ");
  
  // The return value for the function
  String stringLocalTime="Failed to obtain NTP time";
  
  // Set the ESP32's internal RTC to NTP Time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  stringLocalTime = getESPTime();
  debugUART.print("    NTP Time: ");
  debugUART.println(stringLocalTime);
  
  if (stringLocalTime != "Failed to obtain NTP time"){      
    // Set the external, battery-backed RTC to the ESP32's time
    if (rtcPresent()){
      debugUART.println("  Synchronizing RTC w/ NTP Time...");
      setRTCTime(); // Set the RTC to the NTP value we just got.
      debugUART.print("    Updated RTC Time: ");
      debugUART.println(getRTCTime()); 
    }
  }  
  
}

//*****************************************************************************
// Format a byte as a left zero-padded, two-digit decimal string
String twoDigits(byte value){
  String message = String(value, DEC);
  if (value <10) {
      message = "0" + message;
  }
  return message;
}

#endif // __DIGAME_TIME_H__
