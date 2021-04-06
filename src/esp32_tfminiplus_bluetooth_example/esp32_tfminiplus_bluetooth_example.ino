/*
 *  TFMini-plus LIDAR example program.
 * Copyright 2021, Digame Systems. All rights reserved.
 */

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include <TFMPlus.h>    // Include TFMini Plus LIDAR Library v1.4.0
#include <WiFi.h>       // WiFi stack

TFMPlus tfmP;           // Create a TFMini Plus object

// Aliases for easier reading
#define debugUART Serial
#define tfMiniUART Serial2   

BluetoothSerial SerialBT;

// Parameters found in CONFIG.TXT on the SD card. 
String stringDeviceName = "Bailee\'s Office";       

//****************************************************************************************
// Return the device's MAC address
String getMACAddress(){
  byte mac[6];

  WiFi.macAddress(mac);
  String retString= String(String(mac[5],HEX)+":");
  
  retString = String(retString + String(mac[4],HEX) +":");
  retString = String(retString + String(mac[3],HEX) +":");  
  retString = String(retString + String(mac[2],HEX) +":");
  retString = String(retString + String(mac[1],HEX) +":");
  retString = String(retString + String(mac[0],HEX));

  return retString;
}



//****************************************************************************************
// Device initialization                                   
//****************************************************************************************
void setup()
{
    debugUART.begin(115200);   // Intialize terminal serial port
    delay(1000);               // Give port time to initalize

    SerialBT.begin("ShuttleCounter"); //Bluetooth device name
    debugUART.println("The device started, now you can pair it with bluetooth!");
    
    debugUART.println("*****************************************************");
    debugUART.println("ParkData LIDAR Sensor Example");
    debugUART.println("Version 1.0");
    debugUART.println("Copyright 2021, Digame Systems. All rights reserved.");
    debugUART.println("*****************************************************");
    debugUART.print("MAC Address: ");
    debugUART.println(getMACAddress());
    delay(5000);

    tfMiniUART.begin(115200);  // Initialize TFMPLus device serial port.
    delay(1000);               // Give port time to initalize
    tfmP.begin(&tfMiniUART);   // Initialize device library object and...
                               // pass device serial port to the object.

    // Send some commands to the TFMini-Plus
    // - - Perform a system reset - -
    debugUART.printf( "Activating LIDAR Sensor... ");
    if( tfmP.sendCommand(SYSTEM_RESET, 0)){
        debugUART.println("Sensor Active.");
    }
    else tfmP.printReply();
    debugUART.println("Running!");  
}


// Initialize some variables
int16_t tfDist = 0;    // Distance to object in centimeters
int16_t tfFlux = 0;    // Strength or quality of return signal
int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
float   smoothed = 0.0;

int lidarUpdateRate = 10; // 100Hz -> 10 ms


// For our primitive people detector
bool iSeeAPersonNow    = false;
bool iSawAPersonBefore = false;
float personSignal = 0.0;
int32_t personCount = 0;


//************************************************************************
//************************************************************************
void loop()
{
  delay(lidarUpdateRate);
  
  // Read the LIDAR Sensor
  if( tfmP.getData( tfDist, tfFlux, tfTemp)) { 

    //Filter the measured distance
    smoothed = smoothed * 0.5 + (float)tfDist * 0.5;


    //Our primitive people detector
    iSeeAPersonNow = (smoothed < 150);

    if ((iSawAPersonBefore == true) && (iSeeAPersonNow == false)){
      personSignal = 200.0;
      personCount += 1;

      String jsonPayload = "{\"deviceName\":\"" + stringDeviceName + 
                       "\",\"deviceMAC\":\"" + getMACAddress() +  
                       "\",\"eventType\":\"person" +
                       "\",\"count\":\"" + personCount + "\"" +
                       "}";

      SerialBT.println(jsonPayload);
      
    } else {
      personSignal = 0.0;
    }

    iSawAPersonBefore = iSeeAPersonNow; 
        
    debugUART.print(tfDist);
    debugUART.print(" ");
    debugUART.print(smoothed);
    debugUART.print(" ");
    debugUART.print(personSignal);
    debugUART.print(" ");
    debugUART.println(personCount);

    /*
    SerialBT.print(tfDist);
    SerialBT.print(" ");
    SerialBT.print(smoothed);
    SerialBT.print(" ");
    SerialBT.print(personSignal);
    SerialBT.print(" ");
    SerialBT.println(personCount);
    */
   
  }
}
