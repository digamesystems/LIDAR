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

### Overview:

At startup, program parameters are read from a configuration file, CONFIG.TXT found on the micro SD card. These may be changed by editing the file. File format is a simple text file contiaining the following parameters: 

    device name  -- User Defined
    distance threshold -- Less than this distance detected by the LIDAR sensor counts as vehicle 'presence' 
    operating mode (currently only opmodeNetwork is supported)
    SSID -- Network name
    password -- Network password
    serverURL -- Address for http POSTs of JSON messages

Once parameters are loaded, the device attempts to connect to the network with the specified credentials. If successful, system time is set from a connection to an NTP server and the real time clock is updated. In the event of a failure, the last known time set in the RTC will be used. The RTC is based on the DS3231 chip which has high accuracy and is temperature compensated. It should be accurate to within a few seconds a year without adjustment.

At this point, the main loop executes and the system makes repeated distance measurments using the LIDAR sensor over UART. Default acquisition rate is 100Hz and the distances measured are accurate to a few centimeters over ranges from 0.1 - 12 meters (under optimal conditions).

Passing vehicles generate a characteristic signature as they travel through the beam which is interpreted by the software. -- Several algorithms for detection are under evaluation including threshold-based and correlation-based approaches.

On the detection of a vehicle, a JSON message is created for delivery to the server URL specified in the configuration. JSON messages are of the following format: 



