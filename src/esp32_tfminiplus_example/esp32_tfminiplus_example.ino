/*
 *  TFMini-plus LIDAR example program.
 * Copyright 2021, Digame Systems. All rights reserved.
 */

#include <TFMPlus.h>    // Include TFMini Plus LIDAR Library v1.4.0
TFMPlus tfmP;           // Create a TFMini Plus object

// Aliases for easier reading
#define debugUART Serial
#define tfMiniUART Serial2   

//****************************************************************************************
// Device initialization                                   
//****************************************************************************************
void setup()
{
    debugUART.begin(115200);   // Intialize terminal serial port
    delay(1000);               // Give port time to initalize
    
    debugUART.println("*****************************************************");
    debugUART.println("ParkData LIDAR Sensor Example");
    debugUART.println("Version 1.0");
    debugUART.println("Copyright 2021, Digame Systems. All rights reserved.");
    debugUART.println("*****************************************************");

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


//************************************************************************
//************************************************************************
void loop()
{
    delay(lidarUpdateRate);
    
    // Read the LIDAR Sensor
    if( tfmP.getData( tfDist, tfFlux, tfTemp)) { 

      //Filter the measured distance
      smoothed = smoothed * 0.99 + (float)tfDist * 0.01;
     
      debugUART.print(tfDist);
      debugUART.print(" ");
      debugUART.println(smoothed);
     
  }

}