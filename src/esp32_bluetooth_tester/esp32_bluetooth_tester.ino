/*
Bluetooth test application

*/


//****************************************************************************************
// Includes
//****************************************************************************************
#include "BluetoothSerial.h" // Part of the ESP32 board package. 
                             // By Evandro Copercini - 2018


// Aliases for easier reading
#define debugUART  Serial

//****************************************************************************************
// Objects
//****************************************************************************************
BluetoothSerial btUART; // Create a BlueTooth Serial Port Object

//****************************************************************************************
// Globals
//****************************************************************************************

//****************************************************************************************
// Device initialization
//****************************************************************************************
void setup()
{
  
  debugUART.begin(115200);        // Intialize terminal serial port
  delay(1000);                    // Give port time to initalize
  
  btUART.begin("ShuttleCounter"); //Bluetooth device name -- TODO: Provide opportunity to change names. 
  delay(1000);                    // Give port time to initalize

  debugUART.println("*****************************************************");
  debugUART.println("ParkData Bluetooth Test Application");
  debugUART.println("Version 1.0");
  debugUART.println("Copyright 2021, Digame Systems. All rights reserved.");
  debugUART.println("*****************************************************");

}
 
int baselineCounter = 0;

//************************************************************************
//************************************************************************
void loop(){

  delay(1000);
  btUART.println(baselineCounter);
  baselineCounter++;

}
