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
//
// Never use a JsonDocument to store the configuration!
// A JsonDocument is *not* a permanent storage; it's only a temporary storage
// used during the serialization phase. See:
// https://arduinojson.org/v6/faq/why-must-i-create-a-separate-config-object/

struct Config 
{
  // Parameters found in PARAMETERS.TXT on the SD card. 
  // Fails over to these values if not found.
  
  String deviceName          = "Digame SJ HQ"; // 
  
  // Network:       
  String ssid                = "AndroidAP3AE2"; //"Bighead";        // Wireless network name. 
  String password            = "ohpp8971";      //"billgates";      // Network PW
  String serverURL           = "https://trailwaze.info/zion/lidar_sensor_import.php";     // The ParkData server URL
  
  // LoRa:
  String loraAddress         = "1";
  String loraNetworkID       = "16";
  String loraBand            = "915000000";
  String loraSF              = "12";
  String loraBW              = "7";
  String loraCR              = "1";
  String loraPreamble        = "4";
  
};


const char *filename = "/parameters.txt";  // <- SD library uses 8.3 filenames

//****************************************************************************************
// See if the card is present and can be initialized.
bool initSDCard(){
  if (!SD.begin(SD_CS)) {
    return false;
  }  else {
    return true;    
  }
}


// Loads the configuration from a file
void loadConfiguration(const char *filename, Config &config) {
  // Open file for reading
  File file = SD.open(filename);

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use https://arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<768> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));

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

  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
}

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
  StaticJsonDocument<768> doc;

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

/*
void setup() {
  // Initialize serial port
  Serial.begin(9600);
  while (!Serial) continue;

  // Initialize SD library
  const int chipSelect = 4;
  while (!SD.begin(chipSelect)) {
    Serial.println(F("Failed to initialize SD library"));
    delay(1000);
  }

  // Should load default config if run for the first time
  Serial.println(F("Loading configuration..."));
  loadConfiguration(filename, config);

  // Create configuration file
  Serial.println(F("Saving configuration..."));
  saveConfiguration(filename, config);

  // Dump config file
  Serial.println(F("Print config file..."));
  printFile(filename);
}
*/

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
// Use the coupon code TWENTY for a 20% discount ???????????????


#endif
