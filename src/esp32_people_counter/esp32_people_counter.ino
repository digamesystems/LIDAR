/*
    Heimdall -- "Heimdall sees all!" 

    "ONE! One person!... TWO! Two people!... THREE! Three people! Ah! Ah! Ah!"
                                                                 -- The Count.
    This program uses a TFMini-plus LIDAR to count people boarding /
    unboarding shuttle busses. Data is reported in JSON format via Bluetooth
    classic. Program parameters are read from an SD card module or hardcoded 
    defaults depending on compile time parameters. 

    LIDAR Sensor:
    http://en.benewake.com/product/detail/5c345cd0e5b3a844c472329b.html
    (See manual in /docs folder.)

    Written for the ESP32 WROOM Dev board V4 
    (Incl. WiFi, Bluetooth, and stacks of I/O.)
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/hw-reference/esp32c3/user-guide-devkitc-02.html

    Copyright 2021, Digame Systems. All rights reserved.

    Testing the Sensor: 
    https://youtu.be/LPxUawpHUEk
    https://youtu.be/7mM1T-jgChU
    https://youtu.be/dQw4w9WgXcQ

*/


//****************************************************************************************
// Includes
//****************************************************************************************
#include "BluetoothSerial.h" // Part of the ESP32 board package. 
                             // By Evandro Copercini - 2018
                        
#include <TFMPlus.h>    // Include TFMini Plus LIDAR Library v1.4.0
                        // https://github.com/budryerson/TFMini-Plus
                        
#include <WiFi.h>       // WiFi stack (Needed to grab the MAC Address)
                        // https://www.arduino.cc/en/Reference/WiFi
                        
#include <SPI.h>        // SPI bus functions to talk to the SD Card
                        // Part of the Arduino Core 

#include <SD.h>         // SD file handling 
                        // https://www.arduino.cc/en/Reference/SD

// Aliases for easier reading
#define debugUART  Serial
#define tfMiniUART Serial2

#define USE_SD_CARD true // A compile-time switch for whether or not we want to include an
                         // SD card in the system.

//****************************************************************************************
// Objects
//****************************************************************************************
BluetoothSerial btUART; // Create a BlueTooth Serial Port Object
TFMPlus tfmP;           // Create a TFMini Plus Object

//****************************************************************************************
// Globals
//****************************************************************************************
//Operating modes. -- Currently only one
const int EXPSMOOTH = 1;

// These will be updated from values in the SD card 
String    stringDeviceName  = "Bailee\'s Office";
float     smoothingCoef     = 0.95;    // Filter parameter. (0-1.0) The closer to 1.0, the smoother / slower the filtered response.

int16_t   rawDistance       = 0;      // Distance to object in centimeters
float     smoothedDistance  = 0.0;    // The filtered value of the raw sensor readings
const int lidarUpdateRate   = 10;     // 100Hz -> 10 ms
float     baseline          = 0.0;    // The distance to the floor...
int16_t   operatingMode     = EXPSMOOTH;

// For our primitive people detector
bool      iSeeAPersonNow    = false;
bool      iSawAPersonBefore = false;
float     personSignal      = 0.0;    // For charting in Serial Plotter
int32_t   personCount       = 0;

// Parameters found in CONFIG.TXT on the SD card.      
String    stringOpMode = "1";          // Gives us the ability to play with algorithms.
String    stringSmoothingCoef= "0.96"; // Filter parameter (See above)

//****************************************************************************************
// Write parameters to the SD Card.
void writeDefaults(){
  debugUART.println("Writing default values...");
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
  File dataFile = SD.open("/CONFIG2.TXT", FILE_WRITE);


  // if the file is available, write to it:
  if (dataFile) {
      dataFile.println(stringDeviceName);
      dataFile.println(stringOpMode);
      dataFile.println(stringSmoothingCoef);
      
      dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    debugUART.println("Error opening CONFIG2.TXT");
  }

}

//****************************************************************************************
// Read parameters from the SD Card.
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
  File dataFile = SD.open("/CONFIG2.TXT");

  // if the file is available, read from to it:
  if (dataFile) {
      stringDeviceName = dataFile.readStringUntil('\r');
      stringDeviceName.trim();
      debugUART.print("Device Name: ");
      debugUART.println(stringDeviceName);

      stringOpMode = dataFile.readStringUntil('\r');
      stringOpMode.trim();
      debugUART.print("Operating Mode: ");
      debugUART.println(stringOpMode);
      operatingMode = stringOpMode.toInt();

      stringSmoothingCoef = dataFile.readStringUntil('\r');
      stringSmoothingCoef.trim();
      debugUART.print("Smoothing Coefficient: ");
      debugUART.println(stringSmoothingCoef);
      smoothingCoef = stringSmoothingCoef.toFloat();
      
      dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    debugUART.println("Error opening CONFIG.TXT");
  }

  delay(3000);
}

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
  debugUART.println("ParkData LIDAR Sensor Example");
  debugUART.println("Version 1.0");
  debugUART.println("Copyright 2021, Digame Systems. All rights reserved.");
  debugUART.println("*****************************************************");

#if USE_SD_CARD
  readDefaults();
  //writeDefaults();
#endif
  
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

    /*
    // Automatic Baseline Code:
    // Give the smoothedDistance a little time to settle in.
    if (baselineCounter<500){
      baselineCounter++;
      if (baselineCounter == 500) baseline = smoothedDistance; //Set our baseline.   
    }
    */
    
    // Hardcoded baseline: 
    baseline = 200; // Set to an appropriate value...
    
    //Filter the measured distance
    smoothedDistance = smoothedDistance * smoothingCoef + (float)rawDistance * (1 - smoothingCoef);

    //Our primitive people detector
    iSeeAPersonNow = (smoothedDistance < baseline * 0.95); 

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
