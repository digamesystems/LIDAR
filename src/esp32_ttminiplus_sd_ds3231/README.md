## esp32_tfminiplus_sd_ds3231.ino

### Description:
This application supports the Digame Systems HEIMDALL traffic counting system. 

### Authors: 
John Price

### System Components:
* [ESP32 WROOM 32D development board](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html)
* One of several compatible LIDAR sensors (see below)
* A [micro SD card breakout module](https://www.adafruit.com/product/254) for system configuration settings and backup.
* A battery backed [real time clock module](https://www.adafruit.com/product/3013)
* [Solar panel](https://voltaicsystems.com/9-watt-panel/)
* Rechargable [battery pack](https://voltaicsystems.com/v50/)
* IP67 Rated Enclosure

### Overview:

At startup, program parameters are read from a configuration file, CONFIG.TXT found on the micro SD card. These may be changed by editing the file. File format is simple text contiaining the following parameters: 

    device name  -- User Defined
    distance threshold -- Less than this distance detected by the LIDAR sensor counts as vehicle 'presence' 
    operating mode (currently only opmodeThreshold and opmodeCorrelation are supported)
    SSID -- Network name
    password -- Network password
    serverURL -- Address for http POSTs of JSON messages

Once parameters are loaded, the device attempts to connect to the network with the specified credentials. If successful, system time is set from a connection to an NTP server and the real time clock is updated. In the event of a failure, the last known time set in the RTC will be used. The RTC is based on the DS3231 chip which has high accuracy and is temperature compensated. It should be accurate to within a few seconds a year without adjustment.

At this point, the main loop executes and the system makes repeated distance measurments using the LIDAR sensor over UART. Default acquisition rate is 100Hz and the distances measured are accurate to a few centimeters over ranges from 0.1 - 12 meters (under optimal conditions).

The software is compatible with several LIDAR sensors from the [Benewake](http://en.benewake.com/) "TF" family: 
* [TFMini-S](http://en.benewake.com/product/detail/5c345e26e5b3a844c472329c.html) 0-12 M Range
* [TFMini-Plus](http://en.benewake.com/product/detail/5c345cd0e5b3a844c472329b.html) 0-12 M Range, IP67
* [TF-Luna](http://en.benewake.com/product/detail/5e1c1fd04d839408076b6255.html) 0-8 M Range, Low Cost
* [TF-03](http://en.benewake.com/product/detail/5c345cc2e5b3a844c472329a.html) 0-180 M Range!, IP67

Passing vehicles generate a characteristic signature as they travel through the beam which is interpreted by the software. -- Several algorithms for detection are under evaluation including threshold-based and correlation-based approaches.

On the detection of a vehicle, a JSON message is created for delivery to the server URL specified in the configuration. JSON messages are of the following format: 

    "{"deviceName": "YourDeviceName",
      "deviceMAC":  "YourMACAddress",
      "timeStamp":" "YourRTCTime (UTC)",
      "eventType":  "<boot|heartbeat|vehicle>"
      }"
      
For "vehicle" events, an optional array of raw LIDAR data maybe included in the payload:
   
    <...pseudo code snippet...>
    
    jsonPayload = jsonPayload + "operatingMode": "<opmodeThreshold|opmodeCorrelation>,"
    jsonPayload = jsonPayload + "rawData": + "["
    using index_t = decltype(buffer)::index_t;
    for (index_t i = 0; i < buffer.size(); i++) {
        jsonPayload = jsonPayload + buffer[i]; 
        if (i<buffer.size()-1){ 
          jsonPayload = jsonPayload + ",";
        }
    }
    //Close out the message
    jsonPayload = jsonPayload + "]}";
          
This allows us to analyze the last second or so of raw data before the sensor claimed that a car had passed.

An attempt is then made to send the data to the server. If this fails, data is saved to the SD card until a successful connection can be reestablished. Reconnect attempts are made every 5 minutes by default. Once a successful connection has been made, the data saved locally is sent to the server and upon completion, the data stored on the SD card is deleted. 
    
    


