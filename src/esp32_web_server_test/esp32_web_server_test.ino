/*********
  Credit to Rui Santos for initial example.
  Complete project details at https://randomnerdtutorials.com 

  Additions by John Price
*********/

// Load Wi-Fi library
#include <WiFi.h>

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;
// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

// We'll be using a struct to hold program parameters...
struct Config 
{
  // Parameters found in PARAMETERS.TXT on the SD card. 
  // Fails over to these values if not found.
  
  String deviceName          = "Digame Systems LoRa Base Station"; // 
  
  // Network:       
  String ssid                = "Bighead";        // Wireless network name. 
  String password            = "billgates";      // Network PW
  String serverURL           = "https://trailwaze.info/zion/lidar_sensor_import.php";     // The ParkData server URL
  
  // LoRa:
  String loraAddress         = "1";
  String loraNetworkID       = "7";
  String loraBand            = "915000000";
  String loraSF              = "12";
  String loraCR              = "1";
  String loraBW              = "7";
  String loraPreamble        = "4";
  
};

Config config;


//****************************************************************************************
// Return the device's MAC address
String getMACAddress(){
  byte mac[6];
  String retString;
  WiFi.macAddress(mac);
  retString = String(String(mac[5],HEX)+":");
  retString = String(retString + String(mac[4],HEX) +":");
  retString = String(retString + String(mac[3],HEX) +":");  
  retString = String(retString + String(mac[2],HEX) +":");
  retString = String(retString + String(mac[1],HEX) +":");
  retString = String(retString + String(mac[0],HEX));

  return retString;
}

//****************************************************************************************
//Extract a value for a query parameter from an HTTP header.
String getParam(String header, String paramName){
  String result= ""; 

  int strStart = header.indexOf(paramName+"=");
  
  if (strStart == -1){
    Serial.println(paramName + " not found.");
    return ("Not found.");  
  }
  strStart = strStart + paramName.length() + 1;

  int strStop = header.indexOf(" HTTP/1.1");

  result = header.substring(strStart,strStop);
  result = result.substring(0,result.indexOf("&"));

  // Clean up the string by removing HTML replacements
  result.replace("+"," ");
  result.replace("%3A",":");
  result.replace("%2F","/");
  
  Serial.println(result);
  
  return result;
}

//****************************************************************************************
//****************************************************************************************
void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(config.ssid);
  WiFi.begin(config.ssid.c_str(), config.password.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

//****************************************************************************************
//****************************************************************************************
void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // Handle any GET queries we know about.
            if (header.indexOf("GET /general") >=0) {
              Serial.println("You are tweaking general parameters.");  
              config.deviceName = getParam(header, "devname");
              
            } else if (header.indexOf("GET /network") >=0) {
              Serial.println("You are tweaking network parameters.");  
              config.ssid = getParam(header,"ssid");
              config.password  = getParam(header,"password");
              config.serverURL = getParam(header,"serverurl");
              
            } else if (header.indexOf("GET /lora") >=0) {
              Serial.println("You are tweaking a LoRa parameter."); 
              config.loraAddress   = getParam(header,"address");
              config.loraNetworkID = getParam(header,"networkid");
              config.loraBand      = getParam(header,"band");
              config.loraSF        = getParam(header,"spreadingfactor");
              config.loraBW        = getParam(header,"bandwidth");
              config.loraCR        = getParam(header,"codingrate");
              config.loraPreamble = getParam(header,"preamble");
            }
                      
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head>");
            client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            client.println("<style>");
            client.println(" html { font-family: Helvetica; display: inline-block; margin: 0px auto;}");// text-align: center;}");
            client.println(" form { margin:10px auto; width: 300px; background-color: #eeeeee; border: solid; padding: 5px 15px;}");
            client.println(" h1 { background-color: #e84122; border: none; color: white; padding: 16px 40px;");
            client.println("    text-decoration: none; text-align: center; margin: 2px;}");
            client.println(" label {width: 250px; text-align: left;clear: both;float:left; margin-right:15px;}");
            client.println(" input {width: 250px;}");
            client.println(" .center {  display: block;  margin-left: auto;  margin-right: auto;  width: 175px;}");
            client.println("</style>");
            client.println("</head>");
            
            // Web Page Heading
            client.println("<body><h1>HEIMDALL VCS</h1>");
            client.println("<hr>");

            client.println("<H2>Base Station Configuration</H2>");
            client.println("<p>Welcome. You can use these forms to update system parameters.</p>");

            client.println("<form action=\"/generalparms\">\n");
            client.println("<H3>General</H3>");
            
            client.println("<label for=\"devname\">Device Name</label>");
            client.println("<input type=\"text\" id=\"devname\" name=\"devname\" value=\""+ config.deviceName + "\"><br><br>");
            client.println("<input type=\"submit\" value=\"Submit\"></form>");

            client.println("<form action=\"/networkparms\">\n");
            client.println("<H3>Network</H3>");
            client.println("<p>NOTE: Changes to network settings will be applied after a reboot of the base station. </p>");
            
            client.println("<label>MAC Address: " + getMACAddress() + "</label><br><br>");
            
            client.println("<label for=\"ssid\">SSID</label>");
            client.println("<input type=\"text\" id=\"ssid\" name=\"ssid\" value=\""+ config.ssid + "\"><br><br>\n");
            
            client.println("<label for=\"password\">Password</label>");
            client.println("<input type=\"text\" id=\"password\" name=\"password\" value=\""+ config.password +"\"><br><br>");
            
            client.println("<label for=\"serverurl\">Server URL</label>");
            client.println("<input type=\"text\" id=\"serverurl\" name=\"serverurl\" value=\""+ config.serverURL + "\"</input><br><br>");
            client.println("<input type=\"submit\" value=\"Submit\"></form>");

            client.println("<form action=\"/loraparms\">\n");
            client.println("<H3>LoRa (Low-Power Long-Range Wireless Link)</H3>");
            
            client.println("<label for=\"address\">Address</label>");
            client.println("<input type=\"text\" id=\"address\" name=\"address\" value=\""+ config.loraAddress + "\"><br><br>");
            
            client.println("<label for=\"networkid\">Network ID</label>");
            client.println("<input type=\"text\" id=\"networkid\" name=\"networkid\" value=\"" + config.loraNetworkID +"\"><br><br>");
            
            client.println("<label for=\"band\">Band</label>");
            client.println("<input type=\"text\" id=\"band\" name=\"band\" value=\""+ config.loraBand +"\"><br><br>");
            
            client.println("<label for=\"spreadingfactor\">Spreading Factor</label>");
            client.println("<input type=\"text\" id=\"spreadingfactor\" name=\"spreadingfactor\" value=\""+ config.loraSF + "\"</input><br><br>");
            
            client.println("<label for=\"bandwidth\">Bandwidth</label>");
            client.println("<input type=\"text\" id=\"bandwidth\" name=\"bandwidth\" value=\""+ config.loraBW +"\"><br><br>");
            
            client.println("<label for=\"codingrate\">Coding Rate</label>");
            client.println("<input type=\"text\" id=\"codingrate\" name=\"codingrate\" value=\"" + config.loraCR + "\"><br><br>");
            
            client.println("<label for=\"preamble\">Preamble</label>");
            client.println("<input type=\"text\" id=\"preamble\" name=\"preamble\" value=\""+ config.loraPreamble +"\"</input><br><br>");
            client.println("<input type=\"submit\" value=\"Submit\"></form>");

            client.println("<img src=http://static1.squarespace.com/static/554a673ae4b0d7d5128155bb/t/5ef63fdf6b62f234d82610a2/1595258253753/?format=150w alt=\"Digame Logo\" class=\"center\">");
            client.println("<p style=\"text-align:center; font-style:italic\">Copyright 2021, Digame Systems. All rights reserved.</p>");

            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
