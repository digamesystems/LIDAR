#ifndef __DIGAME_JSON_CONFIG_H__
#define __DIGAME_JSON_CONFIG_H__

#define ADAFRUIT_EINK_SD true  // Some Adafruit Eink Displays have an integrated SD card so we don't need a separate module

#include <SPI.h>               // SPI bus functions to talk to the SD Card
#include <SD.h>                // SD file handling

#if ADAFRUIT_EINK_SD
  #define SD_CS       14       // SD card chip select
#else
  #define SD_CS       4
#endif

#include <ArduinoJson.h>

// Our configuration structure.
struct Config 
{
  // Parameters found in PARAMETERS.TXT on the SD card. 
  // Fails over to these values if not found.
  
  String deviceName          = "YOUR_DEVICE_NAME"; // 
  
  // Network:       
  String ssid                = "YOUR_SSID";     // Wireless network name. 
  String password            = "YOUR_PASSWORD"; // Network PW
  String serverURL           = "https://trailwaze.info/zion/lidar_sensor_import.php"; // The ParkData server URL
  
  // LoRa:
  String loraAddress         = "0";
  String loraNetworkID       = "7";
  String loraBand            = "915000000";
  String loraSF              = "7";
  String loraBW              = "7";
  String loraCR              = "1";
  String loraPreamble        = "7";

  // LIDAR Parameters:
  String lidarDetectionAlgorithm = "Threshold";
  String lidarUpdateInterval     = "10";
  String lidarSmoothingFactor    = "0.6";
  String lidarDistanceThreshold  = "300";
  
};


const char *filename = "/params.txt";  // <- SD library uses 8.3 filenames

void loadConfiguration(const char *filename, const Config &config);
void saveConfiguration(const char *filename, const Config &config);
void printFile(const char *filename);


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
// Loads the configuration from a file
void loadConfiguration(const char *filename, Config &config) {
  // Open file for reading
  File file = SD.open(filename);
  //saveConfiguration(filename, config);
  if (!file) {
    Serial.println(F("Failed to open file. Creating it from default parameters."));
    saveConfiguration(filename, config);
    Serial.println("Trying again...");
    file = SD.open(filename);
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use https://arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<2048> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  
  if (error){
    Serial.println(F("Failed to parse file, using default configuration"));
    file.close();
    return;
  }
  
  // Copy values from the JsonDocument to the Config
  config.deviceName    = (const char*)doc["name"];
  config.ssid          = (const char*)doc["network"]["ssid"];
  config.password      = (const char*)doc["network"]["password"];
  config.serverURL     = (const char*)doc["network"]["serverURL"];
  config.loraAddress   = (const char*)doc["lora"]["address"];
  config.loraNetworkID = (const char*)doc["lora"]["networkID"];
  config.loraBand      = (const char*)doc["lora"]["band"];
  config.loraSF        = (const char*)doc["lora"]["spreadingFactor"];
  config.loraBW        = (const char*)doc["lora"]["bandwidth"];
  config.loraCR        = (const char*)doc["lora"]["codingRate"];
  config.loraPreamble  = (const char*)doc["lora"]["preamble"]; 
  config.lidarDetectionAlgorithm  = (const char*)doc["lidar"]["detectionAlgorithm"];
  config.lidarUpdateInterval      = (const char*)doc["lidar"]["updateInterval"];
  config.lidarSmoothingFactor     = (const char*)doc["lidar"]["smoothingFactor"];
  config.lidarDistanceThreshold   = (const char*)doc["lidar"]["distanceThreshold"]  ;
   
  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
}


//****************************************************************************************
// Saves the configuration to a file
void saveConfiguration(const char *filename, const Config &config) {
  // Delete existing file, otherwise the configuration is appended to the file
  SD.remove(filename);

  // Open file for writing
  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use https://arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<2048> doc;

  // Copy values from the Config struct to the JsonDocument
  doc["name"]                    = config.deviceName;
  doc["network"]["ssid"]         = config.ssid;
  doc["network"]["password"]     = config.password;
  doc["network"]["serverURL"]    = config.serverURL;
  doc["lora"]["address"]         = config.loraAddress;
  doc["lora"]["networkID"]       = config.loraNetworkID;
  doc["lora"]["band"]            = config.loraBand;
  doc["lora"]["spreadingFactor"] = config.loraSF;
  doc["lora"]["bandwidth"]       = config.loraBW;
  doc["lora"]["codingRate"]      = config.loraCR;
  doc["lora"]["preamble"]        = config.loraPreamble; 
  doc["lidar"]["detectionAlgorithm"] = config.lidarDetectionAlgorithm;
  doc["lidar"]["updateInterval"]     = config.lidarUpdateInterval;
  doc["lidar"]["smoothingFactor"]    = config.lidarSmoothingFactor;
  doc["lidar"]["distanceThreshold"]  = config.lidarDistanceThreshold;
  
  // Serialize JSON to file
  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }

  // Close the file
  file.close();
}

// Prints the content of a file to the Serial
void printFile(const char *filename) {
  // Open file for reading
  File file = SD.open(filename);
  if (!file) {
    Serial.println(F("Failed to read file"));
    return;
  }

  // Extract each characters by one by one
  while (file.available()) {
    Serial.print((char)file.read());
  }
  Serial.println();

  // Close the file
  file.close();
}


// Performance issue?
// ------------------
//
// File is an unbuffered stream, which is not optimal for ArduinoJson.
// See: https://arduinojson.org/v6/how-to/improve-speed/

// See also
// --------
//
// https://arduinojson.org/ contains the documentation for all the functions
// used above. It also includes an FAQ that will help you solve any
// serialization or deserialization problem.
//
// The book "Mastering ArduinoJson" contains a case study of a project that has
// a complex configuration with nested members.
// Contrary to this example, the project in the book uses the SPIFFS filesystem.
// Learn more at https://arduinojson.org/book/
// Use the coupon code TWENTY for a 20% discount ❤❤❤❤❤


#endif
