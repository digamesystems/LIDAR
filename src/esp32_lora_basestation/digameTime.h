
/**
 * @file digameTime.h
 *
 * Time and Utility functions for working with clocks.
 */
#include "time.h"       // UTC functions
#include <DS3231.h>     // Real Time Clock Library


#ifndef __DIGAME_TIME_H__
#define __DIGAME_TIME_H__

#define debugUART Serial

bool Century=false;
byte Year;
byte Month;
byte Date;
byte DoW;
byte Hour;
byte Minute;
byte Second; 

bool h12; //Hour counter
bool PM;

// Hardcoded time parameters. -- TODO: get some fallover time servers...
const char* ntpServer = "pool.ntp.org"; // Time server.
const long  gmtOffset_sec = 0;          // No offsets -- We're using GMT
const int   daylightOffset_sec = 0;


//****************************************************************************************
// Silly Utility Function to format time values
// Format a byte as a left zero-padded, two-digit decimal string
String two_digits(byte value){
  String message = String(value, DEC);
  if (value <10) {
      message = "0" + message;
  }
  return message;
}

//****************************************************************************************
// For us, Local Time will be UTC set by the NTP connection in setup.
String getLocalTime()
{
  String retStr;
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    retStr = String("Failed to obtain time");
    return retStr;
  }
  // Jared would like: 
  //YYYY-MM-DD HH:MM:SS
  //debugUART.print(&timeinfo, "%Y-%m-%d %H:%M:%S");
  retStr = String(timeinfo.tm_year+1900); //, "%Y-%m-%d %H:%M:%S");
  retStr = retStr + ("-");
  retStr = retStr + two_digits(timeinfo.tm_mon+1);
  retStr = retStr + ("-");
  retStr = retStr + two_digits(timeinfo.tm_mday);
  retStr = retStr + (" ");
  retStr = retStr + two_digits(timeinfo.tm_hour);
  retStr = retStr + (":");
  retStr = retStr + two_digits(timeinfo.tm_min);
  retStr = retStr + (":");
  retStr = retStr + two_digits(timeinfo.tm_sec);
  return retStr;
}

//****************************************************************************************
int getRTCSecond(){
    RTClib myRTC;
    DateTime now = myRTC.now();
    return now.second(); 
}

//****************************************************************************************
int getRTCMinute(){
    RTClib myRTC;
    DateTime now = myRTC.now();
    return now.minute(); 
}

//****************************************************************************************
int getRTCHour(){
    RTClib myRTC;
    DateTime now = myRTC.now();
    return now.hour(); 
}

//****************************************************************************************
// Retrieve the time from the RTC and format 
String getRTCTime(){
  
    //RTClib myRTC;
    DS3231 clock;
    String message = "";

/*
    DateTime now = myRTC.now();
    message += now.year();
    message += "-";
    message += two_digits(now.month());
    message += "-";
    message += two_digits(now.day());
    message +=" ";
    message += two_digits(now.hour());
    message += ':';
    message += two_digits(now.minute());
    message += ':';
    message += two_digits(now.second());
*/
    message += clock.getYear();
    message += "-";
    bool cent;
    message += two_digits(clock.getMonth(cent));
    message += "-";
    message += two_digits(clock.getDate());
    message +=" ";
    bool hr;
    bool amPM;
    message += two_digits(clock.getHour(hr,amPM));
    message += ':';
    message += two_digits(clock.getMinute());
    message += ':';
    message += two_digits(clock.getSecond());  

    //Serial.println(clock.getTemperature());
    return message;   
}

//****************************************************************************************
bool initRTC(){
  String tNow;

    tNow = getRTCTime();
    return !(tNow == "2165-165-165 165:165:85"); // What the library returns when we don't have an SD card hooked up.
}

//****************************************************************************************
// Attempt to synchronize the RTC with the value set from the NTP synch at boot. 
// If the NTP was not set. Don't do anything and leave the current value in the RTC.
int setRTCTime(){
  
  String retStr;
  struct tm timeinfo;
  
  if(!getLocalTime(&timeinfo)){ // Read from the ESP32 RTC (not battery backed...) 
    retStr = String("Failed to obtain NTP time");
    return 1;
  }

  DS3231 clock;
  clock.setYear(timeinfo.tm_year-100);
  clock.setMonth(timeinfo.tm_mon+1);
  clock.setDate(timeinfo.tm_mday);
  clock.setHour(timeinfo.tm_hour);
  clock.setMinute(timeinfo.tm_min);
  clock.setSecond(timeinfo.tm_sec);
  
  return 0;
}  

//****************************************************************************************
//Init and get the time from the NTP server
bool synchTime(bool rtcPresent){
  debugUART.println("Getting time from NTP Server... ");
  //Init and get the time from the NTP server
  String stringLocalTime="Failed to obtain NTP time";
  
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  stringLocalTime = getLocalTime();
  debugUART.print("  NTP Time: ");
  debugUART.println(stringLocalTime);
  if (stringLocalTime != "Failed to obtain NTP time"){  
    if (rtcPresent){
      debugUART.println("Synchronizing RTC w/ NTP Time...");
      setRTCTime(); // Set the RTC to the NTP value we just got.
      debugUART.print("  Updated RTC Time: ");
      debugUART.println(getRTCTime()); 
    }
  }  
}


#endif // __DIGAME_TIME_H__
