#ifndef __DIGAME_CONFIG_H__
#define __DIGAME_CONFIG_H__

/* Program parameters from the SD card module.
 *  
 */

 
#include <SPI.h>               // SPI bus functions to talk to the SD Card
#include <SD.h>                // SD file handling


#if ADAFRUIT_EINK_SD
  #define SD_CS       14  // SD card chip select
#else
  #define SD_CS       4
#endif

struct programParameters 
{
  // Parameters found in CONFIG.TXT on the SD card. Fails over to these values if not found.
  String deviceName          = "Digame Systems"; //        
  String distThreshold       = "300";            // Lane width parameter for counting
  String linkMode            = "WiFi";           // Currently only opmodeNetwork is supported
  String detAlgorithm        = "Threshold";
  String networkName         = "Bighead";        // Wireless network name. 
  String password            = "billgates";      // Network PW
  String serverURL           = "https://trailwaze.info/zion/lidar_sensor_import.php";     // The ParkData server URL
  float  distanceThreshold   = 300.0;            // Maximum distance at which an object counts as 'present'
};

programParameters params;

//****************************************************************************************
// See if the card is present and can be initialized.
bool initSDCard(){
  if (!SD.begin(SD_CS)) {
    return false;
  }  else {
    return true;    
  }
}

//****************************************************************************************
// Debug dump of current values for program defaults.
void showDefaults(){
  
  debugUART.print("      Device Name: ");
  debugUART.println(params.deviceName);
  
  debugUART.print("      Distance Threshold: ");
  debugUART.println(params.distThreshold);
  params.distanceThreshold = params.distThreshold.toFloat();
  
  debugUART.print("      Link Mode: ");
  debugUART.println(params.linkMode);

  debugUART.print("      Det Algorithm: ");
  debugUART.println(params.detAlgorithm);
 
  debugUART.print("      Network Name: ");
  debugUART.println(params.networkName);
  
  debugUART.print("      Password: ");
  debugUART.println(params.password);
  
  debugUART.print("      Server URL: ");
  debugUART.println(params.serverURL);

}


//****************************************************************************************
// Grab program parameters from the SD Card.
bool readDefaults(){

  File dataFile = SD.open("/CONFIG.TXT");

  // If the file is available, read from it:
  if (dataFile) {
      params.deviceName = dataFile.readStringUntil('\r');
      params.deviceName.trim();
      
      params.distThreshold = dataFile.readStringUntil('\r');
      params.distThreshold.trim();
      params.distanceThreshold = params.distThreshold.toFloat();

      params.linkMode = dataFile.readStringUntil('\r');
      params.linkMode.trim();

      params.detAlgorithm = dataFile.readStringUntil('\r');
      params.detAlgorithm.trim();
      
      params.networkName = dataFile.readStringUntil('\r');
      params.networkName = "AndroidAP3AE2";   // ***********************************REMOVE LATER!!!
      params.networkName.trim();
            
      params.password = dataFile.readStringUntil('\r');
      params.password= "ohpp8971";    //*************************************REMOVE LATER!!!
      params.password.trim();
      
      params.serverURL = dataFile.readStringUntil('\r');
      params.serverURL.trim();
      
      dataFile.close();
      return true;
  }
  // If the file isn't open, pop up an error:
  else {
    debugUART.println("      ERROR! Trouble opening CONFIG.TXT");
    return false;
  }
}
#endif
