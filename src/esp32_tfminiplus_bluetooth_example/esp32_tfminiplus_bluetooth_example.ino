/*
    People-Counter

    "ONE! One person!... TWO! Two people!... THREE! Three people! Ha! Ha! Ha!"
                                                                 -- The Count.

    This program uses a TFMini-plus LIDAR to count people boarding /
    unboarding shuttle busses. Data is reported in JSON format via Bluetooth
    classic.

    http://en.benewake.com/product/detail/5c345cd0e5b3a844c472329b.html
    (See manual in /docs folder.)

    Written for the ESP32 WROOM Dev board V4 (Incl. WiFi, Bluetooth, and stacks
    of I/O.)

    Copyright 2021, Digame Systems. All rights reserved.
*/


//****************************************************************************************
// Includes
//****************************************************************************************
#include "BluetoothSerial.h"

// This came along with the example code. TODO: move into BluetoothSerial.h?
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include <TFMPlus.h>    // Include TFMini Plus LIDAR Library v1.4.0
#include <WiFi.h>       // WiFi stack

// Aliases for easier reading
#define debugUART  Serial
#define tfMiniUART Serial2


//****************************************************************************************
// Objects
//****************************************************************************************

BluetoothSerial btUART; // Create a BlueTooth Serial Port Object
TFMPlus tfmP;           // Create a TFMini Plus Object


//****************************************************************************************
// Device initialization
//****************************************************************************************
void setup()
{
  debugUART.begin(115200);   // Intialize terminal serial port
  delay(1000);               // Give port time to initalize

  btUART.begin("ShuttleCounter"); //Bluetooth device name
  delay(1000);               // Give port time to initalize

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


// Initialize variables. TODO: Move magic values to flash / SD?
String  stringDeviceName = "Bailee\'s Office";
int16_t rawDistance = 0;       // Distance to object in centimeters
float   smoothedDistance = 0.0;     // The filtered value of the raw sensor readings
float   distanceThreshold = 190.0; // Closer than this counts as a person being present
float   smoothingCoef = 0.9;// Filter parameter. (0-1.0) The closer to 1.0, the smoother / slower the filtered response.
int     lidarUpdateRate = 10; // 100Hz -> 10 ms

// For our primitive people detector
bool    iSeeAPersonNow    = false;
bool    iSawAPersonBefore = false;
float   personSignal      = 0.0; // For charting in Serial Plotter
int32_t personCount       = 0;


//************************************************************************
//************************************************************************
void loop()
{
  delay(lidarUpdateRate);

  // Read the LIDAR Sensor
  if ( tfmP.getData(rawDistance)) {

    //Filter the measured distance
    smoothedDistance = smoothedDistance * smoothingCoef + (float)rawDistance * (1 - smoothingCoef);

    //Our primitive people detector
    iSeeAPersonNow = (smoothedDistance < distanceThreshold);

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

    debugUART.print(rawDistance);
    debugUART.print(" ");
    debugUART.print(smoothedDistance);
    debugUART.print(" ");
    debugUART.print(personSignal);
    debugUART.print(" ");
    debugUART.println(personCount);
  }
}
