## esp32_people_counter

### Description: 

A LIDAR-based sensor system for detecting people boarding and unboarding shuttle buses. Events are reported via Bluetooth Classic to a networked tablet that relays the information to a server. 

### Authors: 
John Price, Bailee Allen

### System Components:
* [ESP32 WROOM 32D development board](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html)
* One of several compatible LIDAR sensors (see below)
* Enclosure

### JSON Bluetooth Payload:

The Sensor reports 'people' events over Bluetooth in JSON format. Examples:`

  {"deviceName":"Bailee's Office","deviceMAC":"60:3a:6a:2a:3:a8","eventType":"person","count":"1"}
  {"deviceName":"Bailee's Office","deviceMAC":"60:3a:6a:2a:3:a8","eventType":"person","count":"2"}
  {"deviceName":"Bailee's Office","deviceMAC":"60:3a:6a:2a:3:a8","eventType":"person","count":"3"}
  {"deviceName":"Bailee's Office","deviceMAC":"60:3a:6a:2a:3:a8","eventType":"person","count":"4"}

### Sensors:
The software is compatible with several LIDAR sensors from the [Benewake](http://en.benewake.com/) "TF" family: 
* [TFMini-S](http://en.benewake.com/product/detail/5c345e26e5b3a844c472329c.html) 0-12 M Range
* [TFMini-Plus](http://en.benewake.com/product/detail/5c345cd0e5b3a844c472329b.html) 0-12 M Range, IP67
* [TF-Luna](http://en.benewake.com/product/detail/5e1c1fd04d839408076b6255.html) 0-8 M Range, Low Cost
* [TF-03](http://en.benewake.com/product/detail/5c345cc2e5b3a844c472329a.html) 0-180 M Range!, IP67
