## Arduino Sketch: 
esp32_tfminiplus_sd_ds3231.ino

### Authors: 
John Price

### Description:
This application supports the Digame Systems HEIMDALL traffic counting system. 

### System Components:
* ESP32 WROOM 32D development board
* One of several compatible LIDAR sensors (see below)
* A micro SD card breakout module for system configuration settings and backup.
* A battery backed real time clock module
* Solar panel
* Rechargable battery pack
* IP67 Rated Enclosure



Program parameters are read from a configuration file, CONFIG.TXT found on an SD breakout board. 

Several LIDAR sensors from Benewake are compatible with this code including the TFMini-S, the TFMini-Plus, the TF-03 and the TF-Luna.



    device name

    distance threshold

    operating mode (currently only opmodeNetwork is supported)

    SSID

    Password
