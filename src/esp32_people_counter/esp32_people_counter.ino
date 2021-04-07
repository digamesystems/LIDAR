/*
    People-Counter 

    "ONE! One person!... TWO! Two people!... THREE! Three people! Ah! Ah! Ah!"
                                                                 -- The Count.

    This program uses a TFMini-plus LIDAR to count people boarding /
    unboarding shuttle busses. Data is reported in JSON format via Bluetooth
    classic. Program parameters are read from an SD card module. 

    http://en.benewake.com/product/detail/5c345cd0e5b3a844c472329b.html
    (See manual in /docs folder.)

    Written for the ESP32 WROOM Dev board V4 
    (Incl. WiFi, Bluetooth, and stacks of I/O.)

    Copyright 2021, Digame Systems. All rights reserved.
*/


//****************************************************************************************
// Includes
//****************************************************************************************
#include "BluetoothSerial.h"
#include <TFMPlus.h>    // Include TFMini Plus LIDAR Library v1.4.0
#include <WiFi.h>       // WiFi stack (Needed to grab the MAC Address)
#include <SPI.h>        // SPI bus functions to talk to the SD Card
#include <SD.h>         // SD file handling

// Aliases for easier reading
#define debugUART  Serial
#define tfMiniUART Serial2

//****************************************************************************************
// Objects
//****************************************************************************************
BluetoothSerial btUART; // Create a BlueTooth Serial Port Object
TFMPlus tfmP;           // Create a TFMini Plus Object

//****************************************************************************************
// Globals
//****************************************************************************************
// TODO: Move magic values to flash / SD
String    stringDeviceName  = "Bailee\'s Office";
int16_t   rawDistance       = 0;      // Distance to object in centimeters
float     smoothedDistance  = 0.0;    // The filtered value of the raw sensor readings
float     distanceThreshold = 190.0;  // Closer than this counts as a person being present
float     smoothingCoef     = 0.95;    // Filter parameter. (0-1.0) The closer to 1.0, the smoother / slower the filtered response.
const int lidarUpdateRate   = 10;     // 100Hz -> 10 ms
float     baseline          = 0.0;    // The distance to the floor...

// For our primitive people detector
bool      iSeeAPersonNow    = false;
bool      iSawAPersonBefore = false;
float     personSignal      = 0.0;    // For charting in Serial Plotter
int32_t   personCount       = 0;

// Parameters found in CONFIG.TXT on the SD card.      
String stringDistThreshold; // Lane width parameter for counting
String stringOpMode;        // Currently only opmodeNetwork is supported
String stringSSID;          // Wireless network name. 
String stringPassword;      // Network PW
String stringServerURL;     // The ParkData server URL

//****************************************************************************************
// Grab parameters from the SD Card.
void readDefaults(){
  
  debugUART.println("Reading default values...");
  debugUART.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin()) {
    debugUART.println("Card failed, or not present");
    // don't do anything more:
    // TODO: Fix this. 'Can't just stop in the real world.
    while (1);
  }
  debugUART.println(" Card initialized.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("/CONFIG.TXT");

  // if the file is available, read from to it:
  if (dataFile) {
      stringDeviceName = dataFile.readStringUntil('\r');
      stringDeviceName.trim();
      debugUART.print("Device Name: ");
      debugUART.println(stringDeviceName);

      stringDistThreshold = dataFile.readStringUntil('\r');
      stringDistThreshold.trim();
      debugUART.print("Distance Threshold: ");
      debugUART.println(stringDistThreshold);
      distanceThreshold = stringDistThreshold.toFloat();

      stringOpMode = dataFile.readStringUntil('\r');
      stringOpMode.trim();
      debugUART.print("Operating Mode: ");
      debugUART.println(stringOpMode);
      
      dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    debugUART.println("Error opening CONFIG.TXT");
  }

}

//****************************************************************************************
// Device initialization
//****************************************************************************************
void setup()
{
  //readDefaults();
  
  debugUART.begin(115200);        // Intialize terminal serial port
  delay(1000);                    // Give port time to initalize

  btUART.begin("ShuttleCounter"); //Bluetooth device name -- TODO: Provide opportunity to change names. 
  delay(1000);                    // Give port time to initalize

  debugUART.println("*****************************************************");
  debugUART.println("ParkData LIDAR Sensor Example");
  debugUART.println("Version 1.0");
  debugUART.println("Copyright 2021, Digame Systems. All rights reserved.");
  debugUART.println("*****************************************************");

  tfMiniUART.begin(115200);  // Initialize TFMPLus device serial port.
  delay(1000);               // Give port time to initalize
  tfmP.begin(&tfMiniUART);   // Initialize device library object and...
                             //  pass device serial port to the object.

  // TFMini-Plus
  // Perform a system reset
  debugUART.printf( "Activating LIDAR Sensor... ");
  if ( tfmP.sendCommand(SYSTEM_RESET, 0)) {
    debugUART.println("Sensor Active.");
  }
  else tfmP.printReply();
  debugUART.println("Running!");
}


/*
 * TODO: Explore multiple algorithms for detection simultaneously... 
 *  Self calibration 
 *  smoothing parameters, 
 *  ...
 *  
 */
 

int baselineCounter = 0;

//************************************************************************
//************************************************************************
void loop()
{
  delay(lidarUpdateRate);

  // Read the LIDAR Sensor
  if (tfmP.getData(rawDistance)) {

    // Give the smoothedDistance a little time to settle in.
    if (baselineCounter<500){
      baselineCounter++;
      if (baselineCounter == 500) baseline = smoothedDistance; //Set our baseline.   
    }
    //Filter the measured distance
    smoothedDistance = smoothedDistance * smoothingCoef + (float)rawDistance * (1 - smoothingCoef);

    //Our primitive people detector
    //iSeeAPersonNow = (smoothedDistance < distanceThreshold);
    iSeeAPersonNow = (smoothedDistance < baseline * 0.95); // No more hard coding of the baseline!

    if ((iSawAPersonBefore == true) && (iSeeAPersonNow == false)) {
      personSignal = 200.0;
      personCount += 1;
      
      String jsonPayload = "{\"deviceName\":\"" + stringDeviceName +
                           "\",\"deviceMAC\":\"" + WiFi.macAddress() +
                           "\",\"eventType\":\"person" +
                           "\",\"count\":\"" + personCount + "\"" +
                           "}";

      btUART.println(jsonPayload);

    } else {
      personSignal = 0.0;
    }

    iSawAPersonBefore = iSeeAPersonNow;
    
    debugUART.print(baseline);
    debugUART.print(" ");
    debugUART.print(rawDistance);
    debugUART.print(" ");
    debugUART.print(smoothedDistance);
    debugUART.print(" ");
    debugUART.print(personSignal);
    debugUART.print(" ");
    debugUART.println(personCount);
  }
}
