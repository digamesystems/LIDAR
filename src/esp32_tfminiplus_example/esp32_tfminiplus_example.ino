/*
 *  TFMini-plus LIDAR example program.
 * Copyright 2021, Digame Systems. All rights reserved.
 */

#include <TFMPlus.h>    // Include TFMini Plus LIDAR Library v1.4.0
#include <Wire.h>       // 
#include "time.h"       // UTC functions
#include <SPI.h>        // SPI bus functions to talk to the SD Card

TFMPlus tfmP;           // Create a TFMini Plus object

const int chipSelect = 4; // For the SPI and SD Card

int LED_BUILTIN = 12; // Our built in indicator LED

// Aliases for easier reading
#define debugUART Serial
#define tfMiniUART Serial2   

//****************************************************************************************
// Some of the most sophisticated code ever written.
void blinkLED(){
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                        // wait for a bit
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(100);  
}

//****************************************************************************************
// Device initialization                                   
//****************************************************************************************
void setup()
{
    // Ready the LED.
    pinMode(LED_BUILTIN, OUTPUT);
    
    debugUART.begin(115200);   // Intialize terminal serial port
    delay(1000);               // Give port time to initalize

    Wire.begin();

    debugUART.println("*****************************************************");
    debugUART.println("ParkData LIDAR Sensor Example");
    debugUART.println("Version 1.0");
    debugUART.println("Copyright 2021, Digame Systems. All rights reserved.");
    debugUART.println("*****************************************************");

    tfMiniUART.begin(115200);  // Initialize TFMPLus device serial port.
    delay(1000);               // Give port time to initalize
    tfmP.begin(&tfMiniUART);   // Initialize device library object and...
                               // pass device serial port to the object.

    // Send some example commands to the TFMini-Plus
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

bool carPresent = false;    
bool lastCarPresent = false; 
int  carEvent = 0;

int lidarUpdateRate = 10;


//************************************************************************
//************************************************************************
void loop()
{
    delay(lidarUpdateRate);
    
    // Read the LIDAR Sensor
    if( tfmP.getData( tfDist, tfFlux, tfTemp)) { 

      //Filter the measured distance
      smoothed = smoothed *0.1 + (float)tfDist * 0.9;
      int intSmoothed = (int) smoothed*10;
      
      debugUART.print(tfDist);
      debugUART.print(" ");
      debugUART.print(smoothed);
      debugUART.print(" ");
      debugUART.print(carEvent);
      debugUART.println();
     
  }

}
