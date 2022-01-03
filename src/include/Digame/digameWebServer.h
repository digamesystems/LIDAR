#ifndef __DIGAME_WEB_SERVER_H__
#define __DIGAME_WEB_SERVER_H__


#include <WiFi.h>             // WiFi stack
#include <HTTPClient.h>       // To post to the ParkData Server
#include <digameJSONConfig.h> // for Config struct that hold s network credentials
#include <digameLoRa.h>       // To allow us to tweak LoRa radio parameters
#include <digameLIDAR.h>      // To give us access to the histogram data

#define debugUART Serial

// Set web server port number to 80
WiFiServer server(80);


//****************************************************************************************
//Extract a value for a query parameter from an HTTP header.
String getQueryParam(String header, String paramName){
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
  //debugUART.println(result);

  // Clean up the string by removing HTML replacements
  result.replace("+"," ");

  result.replace("%21","!");
  result.replace("%22","\"");
  result.replace("%23","#");
  result.replace("%24","$");
  result.replace("%25","%");
  result.replace("%26","&");
  result.replace("%27","\'");
  result.replace("%28","(");
  result.replace("%29",")");
  result.replace("%2B","+");
  result.replace("%2C",",");
  result.replace("%2F","/");
    
  result.replace("%3A",":");
  result.replace("%3B",";"); 
  result.replace("%3C","<");   
  result.replace("%3D","="); 
  result.replace("%3E",">");  
  result.replace("%3F","?");  
  
  result.replace("%40","@");

  result.replace("%5B","[");  
  result.replace("%5D","]");  
  result.replace("%5E","^");
  
  result.replace("%60","`");  

  result.replace("%7B","{");  
  result.replace("%7D","}");  
  result.replace("%7E","~");  
  
  
  
  //Serial.println(result);
  result.trim();

  return result;
}

void redirectHome(WiFiClient client){
  client.print("<HEAD>");
  client.print("<meta http-equiv=\"refresh\" content=\"0;url=/\">");
  client.print("</head>");
}

//****************************************************************************************
void processWebClient(String deviceType, WiFiClient client, Config& config){
  
  // Variable to store the HTTP request
  String header;
  // Current time
  unsigned long currentMillis = millis();
  // Previous time
  unsigned long previousMillis = 0; 
  // Define timeout time in milliseconds (example: 2000ms = 2s)
  const long timeoutMillis = 2000;



  if (client) {                             // If a new client connects,
    currentMillis = millis();
    previousMillis = currentMillis;
    if (showDataStream == false){
      Serial.println("New Client.");        // print a message out in the serial port
    } 
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentMillis - previousMillis <= timeoutMillis) {  // loop while the client's connected
      currentMillis = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        if (showDataStream == false){
          Serial.write(c);                    // print it out the serial monitor
        }
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            if (header.indexOf("GET /histo")>=0) {
              client.println("Content-type:text/plain");
            } else if (header.indexOf("GET /clearhisto")>=0){
              client.println("Content-type:text/plain");
            } else{
              client.println("Content-type:text/html");
            }
            client.println("Connection: close");
            client.println();

            // Handle any GET queries we know about.
            if (header.indexOf("GET /counterreset") >=0) {
              //Serial.println("Resetting the Counter.");  
              config.lidarZone1Count = "0";
              //saveConfiguration(filename,config);
              redirectHome(client);
              
            } else if (header.indexOf("GET /general") >=0) {

              //Serial.println("You are tweaking general parameters.");  
              config.deviceName = getQueryParam(header, "devname");
              saveConfiguration(filename,config);

              if (deviceType=="counter"){
                if (getQueryParam(header, "streaming" ) == "ON") { 
                  showDataStream = true;
                } else {
                  showDataStream = false;
                }
              }

              if (getQueryParam(header, "reboot") == "true"){
                debugUART.println("REBOOT Requested!");
                delay(1000);
                ESP.restart();

              }

              redirectHome(client);
              
            } else if (header.indexOf("GET /network") >=0) {
              //Serial.println("You are tweaking network parameters.");  
              config.ssid = getQueryParam(header,"ssid");
              config.password  = getQueryParam(header,"password");
              config.serverURL = getQueryParam(header,"serverurl");
              saveConfiguration(filename,config);
              redirectHome(client);

            } else if (header.indexOf("GET /lora") >=0) {
              //Serial.println("You are tweaking LoRa parameters."); 
              config.loraAddress   = getQueryParam(header,"address");
              config.loraNetworkID = getQueryParam(header,"networkid");
              config.loraBand      = getQueryParam(header,"band");
              config.loraSF        = getQueryParam(header,"spreadingfactor");
              config.loraBW        = getQueryParam(header,"bandwidth");
              config.loraCR        = getQueryParam(header,"codingrate");
              config.loraPreamble = getQueryParam(header,"preamble");
              saveConfiguration(filename,config);
              #if USE_LORA 
              initLoRa();
              configureLoRa(config);
              #endif
              redirectHome(client);

            } else if (header.indexOf("GET /sensors") >=0){
              //Serial.println("You are tweaking Sensor parameters.");
              config.sens1Name  = getQueryParam(header,"sens1name");
              config.sens1Addr  = getQueryParam(header,"sens1addr");
              config.sens1MAC   = getQueryParam(header,"sens1mac");             
              config.sens2Name  = getQueryParam(header,"sens2name");
              config.sens2Addr  = getQueryParam(header,"sens2addr");
              config.sens2MAC   = getQueryParam(header,"sens2mac"); 
              config.sens3Name  = getQueryParam(header,"sens3name");
              config.sens3Addr  = getQueryParam(header,"sens3addr");
              config.sens3MAC   = getQueryParam(header,"sens3mac"); 
              config.sens4Name  = getQueryParam(header,"sens4name");
              config.sens4Addr  = getQueryParam(header,"sens4addr");
              config.sens4MAC   = getQueryParam(header,"sens4mac"); 
              saveConfiguration(filename,config);  
              redirectHome(client);

            } else if (header.indexOf("GET /lidar") >=0) {
              //Serial.println("You are tweaking LIDAR parameters."); 

              config.lidarZone1Min        = getQueryParam(header,"zone1min");
              config.lidarZone1Max        = getQueryParam(header,"zone1max");
              config.lidarZone2Min        = getQueryParam(header,"zone2min");
              config.lidarZone2Max        = getQueryParam(header,"zone2max");
              //config.lidarSmoothingFactor = getQueryParam(header,"smoothfactor");
              //config.lidarResidenceTime   = getQueryParam(header,"residencetime");

              saveConfiguration(filename,config);
              redirectHome(client);

            } else if (header.indexOf("GET /histograph")>=0){
            client.println(getDistanceHistogramChartString(config));
            break;

            }
            else if (header.indexOf("GET /histo")>=0){
            client.println(getDistanceHistogramString());
            break;

            }
             else if (header.indexOf("GET /clearhisto")>=0){
            clearLIDARDistanceHistogram();
            client.println(getDistanceHistogramString());
            break;
            }
            


                      
            // Display the HTML page
            client.println("<!DOCTYPE html><html>");
            client.println("<head>");

            client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");

            client.println(" <style>");

            client.println("  html { font-family: Helvetica; display: inline-block; margin: 0px auto; }");
            client.println("  form { margin:10px auto; width: 300px; background-color: #eeeeee; border: solid; padding: 5px 15px; }");
            client.println("  h1 { background-color: #ac0014; border: none; color: white; padding: 16px 40px;");  
            client.println("      text-decoration: none; text-align: center; margin: 2px; }");
            client.println("  h2 { margin: auto; text-align: center; }");
            client.println("  h3 { margin: auto; text-align: center; font-size: 1.25em; }");
            client.println("  label { width: 150px; font-weight:bold; font-style:italic; text-align: left; clear: both; float:left; margin-right:15px; }");
            client.println("  small { font-weight:normal;}");
            
            client.println("  img { display:block; width:175px; margin-left:auto; margin-right:auto; }");
            client.println("  input[type=\"text\"] { width: 280px; }");
            client.println("  input[type=\"password\"] { width: 280px; }");
            
            client.println("  input[type=\"submit\"] { width: 100px; }");

            //client.println("table, th, td { border: 1px solid black; border-collapse: collapse;}");

            client.println("  table {border-spacing: 0 5px;}");
            //client.println("  th { text-align: left;}");
            client.println("  td { text-align: center;}");
            client.println("  table input[type=\"text\"] { width: 120px; }");
            client.println("  table label { width: 150px; }");
            client.println("  table input[type=\"text\"].narrow { width: 30px; }");

            client.println("  .center {");
            client.println("    position: absolute;");
            client.println("    left: 50%;");
            client.println("    -ms-transform: translate(-50%, -100%);");
            client.println("    transform: translate(-50%, -100%);");
            client.println("  }");

            client.println(" </style>");
            client.println("</head>");
            
            // Web Page Heading
            client.println("<body><h1>HEIMDALL VCS</h1><br>");
            client.println("<h3><em>A Next-Generation Traffic Counting Platform</em></h3>");
            client.println("<h3><em>from D&#237game Systems</em></h3>");
            client.println("<hr>");

            if (deviceType == "basestation"){
                client.println("<br><H2>BASE STATION <br><em>"+ config.deviceName +"</em></H2>");
                client.println("<p>Welcome. You can use these forms to update system parameters for this LoRa-WiFi base station.</p>");

            } else if (deviceType == "counter"){
                client.println("<br><H2>VEHICLE COUNTER <br><em>"+ config.deviceName +"</em></H2>");
               client.println("<p>Welcome. You can use these forms to update system parameters for this LIDAR vehicle counter.</p>");

            }
            
            // COUNTER DATA
            if (deviceType == "counter"){
              client.println("<form action=\"/counterreset\">\n");
              client.println("<H3>Counter Value</H3>");
              client.println("<input type=\"hidden\" id=\"counterval\" name=\"counterval\" value=\""+ config.lidarZone1Count + "\"><br>");
              client.print("<H1>");
              client.print(config.lidarZone1Count);
              client.println("</H1>");
              client.println("<input type=\"hidden\" id=\"counterval2\" name=\"counterval2\" value=\""+ config.lidarZone1Count + "\"><br><br>");
              client.println("<div class=\"center\">");
              client.println("  <input type=\"submit\" value=\"Clear Counter\"></form>");
              client.println("</div>");
            }
            // GENERAL PARAMETERS
            client.println("<form action=\"/generalparms\">\n");
            client.println("<H3>General</H3><br>");
            client.println("<label for=\"devname\">Device Name</label>");
            client.println("<input type=\"text\" id=\"devname\" name=\"devname\" value=\""+ config.deviceName + "\"><br>");

            if (deviceType =="counter"){
              client.println("<br><label for=\"streaming\">Streaming</label>");

              if (showDataStream){
                client.println("<div><br><input type=\"radio\" id=\"true\" name=\"streaming\" value=\"ON\" checked>");            
                client.println("<label for=\"true\"><small>ON</small></label><br>");
                
                client.println("<input type=\"radio\" id=\"false\" name=\"streaming\" value=\"OFF\">");
                client.println("<small><label for=\"false\"><small>OFF</small></label></small><br></div>");

              } else {
                client.println("<div><br><input type=\"radio\" id=\"true\" name=\"streaming\" value=\"ON\">");            
                client.println("<small><label for=\"true\"><small>ON</small></label></small><br>");
              
                client.println("<input type=\"radio\" id=\"false\" name=\"streaming\" value=\"OFF\" checked>");
                client.println("<small><label for=\"false\"><small>OFF</small></label></small><br></div>");
              }
            }
            
            
            client.println("<br><label>Model</label>");
            if (deviceType == "basestation"){
                client.println("<br><small>DS-VC-BASE-LOR-1<br>(Single-Channel, LoRa-WiFi Gateway)</small><br>");
            } else if (deviceType =="counter"){
              #ifdef USE_WIFI
                client.println("<br><small><em>DS-VC-LIDAR-WIFI-1</em><br>(LIDAR Traffic Counter with WiFi Back Haul)</small><br>");
              #else
                client.println("<br><small><em>DS-VC-LIDAR-LOR-1<br>(LIDAR Traffic Counter with LoRa Back Haul)</small><br>");
              #endif
            }

            client.print("<br><label>Software Version<br></label><br><small>");
            client.print(SW_VERSION);
            client.println("</small><br>");

            client.print("<br><label for=\"reboot\">Reboot</label>");
            client.print("<input type=\"checkbox\" id=\"reboot\" name=\"reboot\" value=\"true\"><p><br></p>");

            
            client.println("<div class=\"center\">");
            client.println("<input type=\"submit\" value=\"Submit\"></form>");
            client.println("</div>");
            

            // NETWORK PARAMETERS
            client.println("<form action=\"/networkparms\">\n");
            client.println("<H3>Network</H3>");
            client.println("<p><em>NOTE: Changes to network settings will be applied after a reboot of the device.</em></p>");
            client.println("<label>MAC Address</label><p><em>" + getMACAddress() + "</em></p>");
            client.println("<label for=\"ssid\">SSID</label>");
            client.println("<input type=\"text\" id=\"ssid\" name=\"ssid\" value=\""+ config.ssid + "\"><br><br>\n");
            client.println("<label for=\"password\">Password</label>");
            client.println("<input type=\"password\" id=\"password\" name=\"password\" value=\""+ config.password +"\"><br><br>");
            client.println("<label for=\"serverurl\">Server URL</label>");
            client.println("<input type=\"text\" id=\"serverurl\" name=\"serverurl\" size=\"40\" value=\""+ config.serverURL + "\"</input><br><br>");
            client.println("<br>");
            client.println("<div class=\"center\">");
            client.println("<input type=\"submit\" value=\"Submit\"></form>");
            client.println("</div>");

#if USE_LORA
            // LORA PARAMETERS
            client.println("<form action=\"/loraparms\">\n");
            client.println("<H3>LoRa<br><small>(Low-Power Long-Range Wireless Link)</small></H3>");
            client.println("<p><em>Strategies for optimizing the parameters for LoRa communication may be found in the product manual. Both the counters and the base station must use the same RF parameters for communication to be successful.</em></p>");
            client.println("<label for=\"address\">LoRa Address</label>");
            
            if (deviceType == "counter"){
              client.println("<input type=\"number\" min=\"10\" max=\"65534\" id=\"address\" name=\"address\" value=\""+ config.loraAddress + "\"><br><br>");            
            } else {
              client.println("<input type=\"number\" min=\"1\" max=\"9\" id=\"address\" name=\"address\" value=\""+ config.loraAddress + "\"><br><br>");            
            }
            client.println("<label for=\"networkid\">Network ID</label>");
            client.println("<input type=\"number\" min=\"0\" max=\"16\" id=\"networkid\" name=\"networkid\" value=\"" + config.loraNetworkID +"\"><br><br>");
            client.println("<label for=\"band\">Band</label>");
            client.println("<input type=\"number\" min=\"850000000\" max=\"950000000\" id=\"band\" name=\"band\" value=\""+ config.loraBand +"\"><br><br>");
            client.println("<label for=\"spreadingfactor\">Spreading Factor</label>");
            client.println("<input type=\"number\" min=\"7\" max=\"12\" id=\"spreadingfactor\" name=\"spreadingfactor\" value=\""+ config.loraSF + "\"</input><br><br>");
            client.println("<label for=\"bandwidth\">Bandwidth</label>");
            client.println("<input type=\"number\" min=\"0\" max=\"9\" id=\"bandwidth\" name=\"bandwidth\" value=\""+ config.loraBW +"\"><br><br>");
            client.println("<label for=\"codingrate\">Coding Rate</label>");
            client.println("<input type=\"number\" min=\"1\" max=\"4\" id=\"codingrate\" name=\"codingrate\" value=\"" + config.loraCR + "\"><br><br>");
            client.println("<label for=\"preamble\">Preamble</label>");
            client.println("<input type=\"number\" min=\"4\" max=\"7\" id=\"preamble\" name=\"preamble\" value=\""+ config.loraPreamble +"\"</input><br><br>");
            client.println("<br>");
            client.println("<div class=\"center\">");
            client.println("<input type=\"submit\" value=\"Submit\"></form>");
            client.println("</div>");
#endif 

            if (deviceType == "counter"){
                // LIDAR PARAMETERS
                client.println("<form action=\"/lidarparms\">\n");
                client.println("<H3>Lane Detection Limits (cm)</H3><br>");
                //client.println("<p>Strategies for optimizing the parameters for LIDAR vehicle detection may be found in the product manual. <TODO: LINK></p>");
                
                //client.println("<label for=\"address\">Smoothing Factor (0-0.9)</label>");
                //client.println("<input type=\"text\" id=\"smoothfactor\" name=\"smoothfactor\" value=\""+ config.lidarSmoothingFactor + "\"><br><br>");
                
                //client.println("<label for=\"address\">Residence Time (ms)</label>");
                //client.println("<input type=\"number\" id=\"residencetime\" name=\"residencetime\" value=\""+ config.lidarResidenceTime + "\"><br><br>");
                

                client.println("<label for=\"address\">Lane 1 Min</label>");
                client.println("<input type=\"number\" min=\"0\" max=\"999\" id=\"zone1min\" name=\"zone1min\" value=\""+ config.lidarZone1Min + "\"><br><br>");
                client.println("<label for=\"address\">Lane 1 Max</label>");
                client.println("<input type=\"number\" min=\"0\" max=\"999\" id=\"zone1max\" name=\"zone1max\" value=\""+ config.lidarZone1Max + "\"><br><br>");
                client.println("<label for=\"address\">Lane 2 Min</label>");
                client.println("<input type=\"number\" min=\"0\" max=\"999\" id=\"zone2min\" name=\"zone2min\" value=\""+ config.lidarZone2Min + "\"><br><br>");
                client.println("<label for=\"address\">Lane 2 Max</label>");
                client.println("<input type=\"number\" min=\"0\" max=\"999\" id=\"zone2max\" name=\"zone2max\" value=\""+ config.lidarZone2Max + "\"><br><br>");
                client.println("<br>");
                
                client.println("<div class=\"center\">");
                client.println("<input type=\"submit\" value=\"Submit\"></form>");
                client.println("</div>");
            }

            if (deviceType == "basestation"){
                // COUNTER / SENSOR PARAMETERS
                client.println("<form action=\"/sensors\">\n");
                client.println("<H3>Sensor Definitions</H3>");
                client.println("<p><em>A base station can support up to four (4) traffic counters. Each counter needs a unique LoRa address to communicate. The MAC address for the counter can be found on the label on the back of the sensor.</em></p>");
                
                client.println("<table>");      
                client.println("<thead><th>Name</th><th>Adr</th><th>MAC Address</th></thead>");
                client.println("<tbody>");
                
                client.println("<tr>");
                client.println("<td><input type=\"text\" id=\"sens1name\" name=\"sens1name\" value=\"" + config.sens1Name + "\"></td>");
                client.println("<td><input class=\"narrow\" type=\"text\" id=\"sens1addr\" name=\"sens1addr\" value=\""+ config.sens1Addr + "\"></td>");
                client.println("<td><input type=\"text\" id=\"sens1mac\" name=\"sens1mac\" value=\"" + config.sens1MAC + "\"></td>");
                client.println("</tr>");

                client.println("<tr>");
                client.println("<td><input type=\"text\" id=\"sens2name\" name=\"sens2name\" value=\"" + config.sens2Name + "\"></td>");
                client.println("<td><input class=\"narrow\" type=\"text\" id=\"sens2addr\" name=\"sens2addr\" value=\""+ config.sens2Addr + "\"></td>");
                client.println("<td><input type=\"text\" id=\"sens2mac\" name=\"sens2mac\" value=\"" + config.sens2MAC + "\"></td>");
                client.println("</tr>");

                client.println("<tr>");
                client.println("<td><input type=\"text\" id=\"sens3name\" name=\"sens3name\" value=\"" + config.sens3Name + "\"></td>");
                client.println("<td><input class=\"narrow\" type=\"text\" id=\"sens3addr\" name=\"sens3addr\" value=\""+ config.sens3Addr + "\"></td>");
                client.println("<td><input type=\"text\" id=\"sens3mac\" name=\"sens3mac\" value=\"" + config.sens3MAC + "\"></td>");
                client.println("</tr>");

                client.println("<tr>");
                client.println("<td><input type=\"text\" id=\"sens4name\" name=\"sens4name\" value=\"" + config.sens4Name + "\"></td>");
                client.println("<td><input class=\"narrow\" type=\"text\" id=\"sens4addr\" name=\"sens4addr\" value=\""+ config.sens4Addr + "\"></td>");
                client.println("<td><input type=\"text\" id=\"sens4mac\" name=\"sens4mac\" value=\"" + config.sens4MAC + "\"></td>");
                client.println("</tr>");          

                client.println("</tbody>");
            
                client.println("</table><br>");
                client.println("<br>");

                client.println("<div class=\"center\">");
                client.println("<input type=\"submit\" value=\"Submit\"></form>");
                client.println("</div>");




                client.println("<form action=\"/sensors\">\n");
                client.println("<H3>Sensor Data</H3><br>");
                
                client.println("<table style=\"width:100%\">");      
                
                client.println("<thead><th style=\"width:50%\">Name</th><th style=\"width:50%\">Events</th></thead>");
                client.println("<tbody>");
                
                client.println("<tr>");
                client.println("<td><em>" + config.sens1Name + "</em></td>");
                client.println("<td>" + str1Count + "</td>");
                client.println("</tr>");

                client.println("<tr>");
                client.println("<td><em>" + config.sens2Name + "</em></td>");
                client.println("<td>" + str2Count + "</td>");
                client.println("</tr>");

                client.println("<tr>");
                client.println("<td><em>" + config.sens3Name + "</em></td>");
                client.println("<td>" + str3Count + "</td>");
                client.println("</tr>");

                client.println("<tr>");
                client.println("<td><em>" + config.sens4Name + "</em></td>");
                client.println("<td>" + str4Count + "</td>");
                client.println("</tr>");

                client.println("</tbody>");
            
                client.println("</table><br>");
                client.println("<br>");

                client.println("</form>");
                



            }
            
            //client.println("<div style=\"text-align: center;\"");
            //client.println("<img src=https://images.squarespace-cdn.com/content/v1/554a673ae4b0d7d5128155bb/1625186042479-70RSPEWTSG8747ZYA8M1/parkdata+logo-01.png?format=150w >");
             client.println("<img src=http://static1.squarespace.com/static/554a673ae4b0d7d5128155bb/t/5ef63fdf6b62f234d82610a2/1595258253753/?format=150w alt=\"Digame Logo\" >");
            //client.println("</div>");
            client.println("<p style=\"text-align:center; font-style:italic\">Copyright 2021, D&#237game Systems. All rights reserved.</p>");
            
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
    //Serial.println("Client disconnected.");
    //Serial.println("");
  }  

}




#endif // __DIGAME_WEB_SERVER_H__