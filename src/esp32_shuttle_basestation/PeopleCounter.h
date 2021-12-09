#ifndef __PEOPLECOUNTER_H__
#define __PEOPLECOUNTER_H__

#include <digameTime.h>   // For RTC functions
#include <ArduinoJson.h>  // JSON (de)Serializer   

#define INBOUND 0
#define OUTBOUND 1

Class PeopleCounter{
  public: 
  String name           = "";
  String macAddress     = "";  
  unsigned int events[] = {0, 0};  // in/out count
                                                              
  void   reset();
  void   update(String jsonMessage);
  String toJSON();
};


Class ShuttleStop{
  public:
  String name = "Unknown";
  String ssid = "";
  String arrivalTime = "";
  String departureTime = "";

  String toJSON();
};

Class RouteReport{
  public:
  String prefix = "{";
  String suffix = "}";
  String jsonReport = "";

  void clear();
  void initialize();
  void append(String str);
  void finalize(); 
  
};

Class ShuttleBus{
  public: 
  String name = "";
  String macAddress = "";
  String knownLocations[] PROGMEM = {"Bighead", 
                                     "Tower88", 
                                     "William Shatner's Toupee", 
                                     "Pretty Fly For A Wi-Fi V4",
                                     "4th_StreetPizzaCo", 
                                     "SanPedroSquareMarket",
                                     "AndroidAP3AE2"}; // Saving in PROGMEM                             
  String counterAddresses[] =       {"ShuttleCounter_5ccc",
                                     "ShuttleCounter_aaaa"};
  
  RouteReport routeReport;
  ShuttleStop currentStop;
  PeopleCounter frontCounter;
  PeopleCounter rearCounter;   

  void connectToCounters();
  void scanForKnownLocations();

};


void PeopleCounter::reset(){
    location         = "Unknown";  
    firstEventTime   = getRTCTime(); 
    lastEventTime    = firstEventTime; 
    events[INBOUND]  = 0;
    events[OUTBOUND] = 0;
}

void PeopleCounter::update(String jsonMessage){
  StaticJsonDocument<2048> counterMessage;  
  deserializeJson(counterMessage, jsonMessage);
  
  lastEventTime = getRTCTime();
  macAddresses = (const char *)counterMessage["deviceMAC"];
   
  if (counterMessage["eventType"] =="inbound"){
    events[INBOUND]++;     
  }else{
    events[OUTBOUND]++;    
  }
}

String PeopleCounter::toJSON(){
  StaticJsonDocument<2048> shuttleStopJSON;
  
  shuttleStopJSON["startTime"]   = firstEventTime;
  shuttleStopJSON["endTime"]     = lastEventTime;
  shuttleStopJSON["macAddress"]  = macAddress;
  shuttleStopJSON["inbound"]     = events[INBOUND];
  shuttleStopJSON["outbound"]    = events[OUTBOUND];
  
  String retValue; 
  serializeJson(shuttleStopJSON, retValue);
  
  return retValue;

}




#endif //__PEOPLECOUNTER_H__
