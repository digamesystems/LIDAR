#ifndef __DIGAME_JSON_CONFIG_H__
#define __DIGAME_JSON_CONFIG_H__

#define ADAFRUIT_EINK_SD true // Some Adafruit Eink Displays have an integrated SD card so we don't need a separate module
#define debugUART Serial

#include <SPI.h> // SPI bus functions to talk to the SD Card
#include <SD.h>  // SD file handling

#if ADAFRUIT_EINK_SD
#define SD_CS 14 // SD card chip select
#else
#define SD_CS 4
#endif

#include <ArduinoJson.h>

// Our configuration structure.
//
struct Config
{
  // Parameters found in PARAMETERS.TXT on the SD card.
  // Fails over to these values if not found.

  String deviceName = "Digame Test Counter"; //"YOUR_DEVICE_NAME"; //

  // Network:
  String ssid = "Bighead";       // "YOUR_SSID";     // Wireless network name.
  String password = "billgates"; // "YOUR_PASSWORD"; // Network PW

  //String serverURL           = "https://trailwaze.info/zion/lidar_sensor_import.php"; // The ParkData server URL
  String serverURL = "http://199.21.201.53/trailwaze/zion/lidar_sensor_import.php"; // http server. Faster!

  // LoRa:
  String loraAddress = "10";
  String loraNetworkID = "7";
  String loraBand = "915000000";
  String loraSF = "10";
  String loraBW = "7";
  String loraCR = "1";
  String loraPreamble = "7";

  // LIDAR Parameters:
  String lidarDetectionAlgorithm = "Threshold";
  String lidarUpdateInterval = "10";
  String lidarSmoothingFactor = "0.6";
  String lidarResidenceTime = "200";
  String lidarZone1Min = "0";
  String lidarZone1Max = "300";
  String lidarZone2Min = "400";
  String lidarZone2Max = "700";
  String lidarZone1Count = "0";
  String lidarZone2Count = "0";

  // Sensors:
  String sens1Addr = "10";
  String sens1Name = "Sensor 1";
  String sens1MAC = "aa:bb:cc:dd:ee:01";

  String sens2Addr = "11";
  String sens2Name = "Sensor 2";
  String sens2MAC = "aa:bb:cc:dd:ee:02";

  String sens3Addr = "12";
  String sens3Name = "Sensor 3";
  String sens3MAC = "aa:bb:cc:dd:ee:03";

  String sens4Addr = "13";
  String sens4Name = "Sensor 4";
  String sens4MAC = "aa:bb:cc:dd:ee:04";

  String displayType = "SSD1608"; // so we can switch displays at run time based on the config file.
};

const char *filename = "/params.txt"; // <- SD library uses 8.3 filenames
const char *histoFilename = "/histo.csv";

bool initSDCard();
void loadConfiguration(const char *filename, Config &config);
void saveConfiguration(const char *filename, Config &config);
void printFile(const char *filename);

// Test for SD Card, Load Config from file
bool initJSONConfig(const char *filename, const Config &config);

//****************************************************************************************
// See if the card is present and can be initialized.
bool initSDCard()
{
  if (!SD.begin(SD_CS))
  {
    return false;
  }
  else
  {
    return true;
  }
}

//****************************************************************************************
// Loads the configuration from a file
void loadConfiguration(const char *filename, Config &config)
{
  // Open file for reading
  debugUART.println("    Opening file for read...");
  File file = SD.open(filename);
  //saveConfiguration(filename, config);
  if (!file)
  {
    Serial.println(F("    Failed to open file. Creating it from default parameters."));
    saveConfiguration(filename, config);
    Serial.println("    Trying again...");
    debugUART.println("    Opening file for read...");
    file = SD.open(filename);
  }

  if (!file)
  {
    debugUART.println("    Load parameters failed!");
    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use https://arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<2048> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);

  if (error)
  {
    Serial.println(F("    Failed to parse file, using default configuration"));
    file.close();
    return;
  }

  // Copy values from the JsonDocument to the Config
  config.deviceName = (const char *)doc["name"];

  config.ssid = (const char *)doc["network"]["ssid"];
  config.password = (const char *)doc["network"]["password"];
  config.serverURL = (const char *)doc["network"]["serverURL"];

  config.loraAddress = (const char *)doc["lora"]["address"];
  config.loraNetworkID = (const char *)doc["lora"]["networkID"];
  config.loraBand = (const char *)doc["lora"]["band"];
  config.loraSF = (const char *)doc["lora"]["spreadingFactor"];
  config.loraBW = (const char *)doc["lora"]["bandwidth"];
  config.loraCR = (const char *)doc["lora"]["codingRate"];
  config.loraPreamble = (const char *)doc["lora"]["preamble"];

  config.lidarDetectionAlgorithm = (const char *)doc["lidar"]["detectionAlgorithm"];
  config.lidarUpdateInterval = (const char *)doc["lidar"]["updateInterval"];
  config.lidarSmoothingFactor = (const char *)doc["lidar"]["smoothingFactor"];
  config.lidarResidenceTime = (const char *)doc["lidar"]["residenceTime"];
  config.lidarZone1Min = (const char *)doc["lidar"]["zone1Min"];
  config.lidarZone1Max = (const char *)doc["lidar"]["zone1Max"];
  config.lidarZone2Min = (const char *)doc["lidar"]["zone2Min"];
  config.lidarZone2Max = (const char *)doc["lidar"]["zone2Max"];

  config.lidarZone1Count = "0"; //(const char *)doc["lidar"]["zone1Count"];
  config.lidarZone2Count = "0"; //(const char *)doc["lidar"]["zone2Count"];


  config.sens1Name = (const char *)doc["sensor"]["1"]["name"];
  config.sens1Addr = (const char *)doc["sensor"]["1"]["addr"];
  config.sens1MAC = (const char *)doc["sensor"]["1"]["mac"];
  config.sens2Name = (const char *)doc["sensor"]["2"]["name"];
  config.sens2Addr = (const char *)doc["sensor"]["2"]["addr"];
  config.sens2MAC = (const char *)doc["sensor"]["2"]["mac"];
  config.sens3Name = (const char *)doc["sensor"]["3"]["name"];
  config.sens3Addr = (const char *)doc["sensor"]["3"]["addr"];
  config.sens3MAC = (const char *)doc["sensor"]["3"]["mac"];
  config.sens4Name = (const char *)doc["sensor"]["4"]["name"];
  config.sens4Addr = (const char *)doc["sensor"]["4"]["addr"];
  config.sens4MAC = (const char *)doc["sensor"]["4"]["mac"];

  config.displayType = (const char *)doc["displayType"];

  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
  printFile(filename);
}

//****************************************************************************************
// Saves the configuration to a file
void saveConfiguration(const char *filename, Config &config)
{
  #if !SHOW_DATA_STREAM
  debugUART.println("    Saving parameters...");
  // Delete existing file, otherwise the configuration is appended to the file

  debugUART.println("    Erasing old file...");
  #endif 

  SD.remove(filename);

  // Open file for writing
  #if !SHOW_DATA_STREAM
  debugUART.println("    Opening file for write...");
  #endif

  File file = SD.open(filename, FILE_WRITE);
  if (!file)
  {
    Serial.println(F("    Failed to create file!"));
    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use https://arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<2048> doc;

  // Copy values from the Config struct to the JsonDocument
  doc["name"] = config.deviceName;
  doc["network"]["ssid"] = config.ssid;
  doc["network"]["password"] = config.password;
  doc["network"]["serverURL"] = config.serverURL;

  doc["lora"]["address"] = config.loraAddress;
  doc["lora"]["networkID"] = config.loraNetworkID;
  doc["lora"]["band"] = config.loraBand;
  doc["lora"]["spreadingFactor"] = config.loraSF;
  doc["lora"]["bandwidth"] = config.loraBW;
  doc["lora"]["codingRate"] = config.loraCR;
  doc["lora"]["preamble"] = config.loraPreamble;

  doc["lidar"]["detectionAlgorithm"] = config.lidarDetectionAlgorithm;
  doc["lidar"]["updateInterval"] = config.lidarUpdateInterval;
  doc["lidar"]["smoothingFactor"] = config.lidarSmoothingFactor;
  doc["lidar"]["residenceTime"] = config.lidarResidenceTime;
  doc["lidar"]["zone1Min"] = config.lidarZone1Min;
  doc["lidar"]["zone1Max"] = config.lidarZone1Max;
  doc["lidar"]["zone2Min"] = config.lidarZone2Min;
  doc["lidar"]["zone2Max"] = config.lidarZone2Max;
  doc["lidar"]["zone1Count"] = config.lidarZone1Count;
  doc["lidar"]["zone2Count"] = config.lidarZone2Count;

  doc["sensor"]["1"]["name"] = config.sens1Name;
  doc["sensor"]["1"]["addr"] = config.sens1Addr;
  doc["sensor"]["1"]["mac"] = config.sens1MAC;
  doc["sensor"]["2"]["name"] = config.sens2Name;
  doc["sensor"]["2"]["addr"] = config.sens2Addr;
  doc["sensor"]["2"]["mac"] = config.sens2MAC;
  doc["sensor"]["3"]["name"] = config.sens3Name;
  doc["sensor"]["3"]["addr"] = config.sens3Addr;
  doc["sensor"]["3"]["mac"] = config.sens3MAC;
  doc["sensor"]["4"]["name"] = config.sens4Name;
  doc["sensor"]["4"]["addr"] = config.sens4Addr;
  doc["sensor"]["4"]["mac"] = config.sens4MAC;

  doc["displayType"] = config.displayType;

  // Serialize JSON to file
  #if !SHOW_DATA_STREAM
  debugUART.println("Writing file...");
  #endif 

  if (serializeJson(doc, file) == 0)
  {
    Serial.println(F("Failed to write to file!"));
  }

  // Close the file
  #if !SHOW_DATA_STREAM
  debugUART.println("Done saving parameters.");
  #endif
  
  file.close();
}

//****************************************************************************************
// Save some text to a file
void saveTextFile(const char *filename, String contents)
{

  //debugUART.println(initSDCard());
  debugUART.print("  Saving data to: ");
  debugUART.println(filename);

  // Delete existing file, otherwise the information is appended to the file
  debugUART.println("    Erasing old file...");
  SD.remove(filename);

  // Open file for writing
  debugUART.println("    Opening file for write...");
  File file = SD.open(filename, FILE_WRITE);

  if (!file)
  {
    debugUART.println(F("    Failed to create file!"));
    return;
  }

  debugUART.println("    Writing file...");
  file.println(contents);

  // Close the file
  debugUART.println("  Done.");
  file.close();
}

//****************************************************************************************
// Prints the content of a file to the Serial
void printFile(const char *filename)
{
  // Open file for reading
  File file = SD.open(filename);
  if (!file)
  {
    Serial.println(F("Failed to read file"));
    return;
  }

  // Extract each characters by one by one
  Serial.print("    ");
  while (file.available())
  {
    Serial.print((char)file.read());
  }
  Serial.println();

  // Close the file
  file.close();
}

//****************************************************************************************
//
bool initJSONConfig(const char *filename, Config &config)
{

  bool sdCardPresent = false;
  debugUART.print("  Testing for SD Card Module... ");
  sdCardPresent = initSDCard();
  if (sdCardPresent)
  {
    debugUART.println("Module found. (Reading parameters from SD Card.)");
    loadConfiguration(filename, config);
    return true;
  }
  else
  {
    debugUART.println("ERROR! Module NOT found. (Parameters set to default values.)");
    return false;
  }
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