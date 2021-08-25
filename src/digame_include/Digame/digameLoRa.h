#ifndef __DIGAME_LORA_H__
#define __DIGAME_LORA_H__

#define debugUART Serial
#define LoRaUART Serial1

#include <ArduinoJson.h>

uint16_t LoRaRetryCount = 0;

//****************************************************************************************
void initLoRa()
{
  LoRaUART.begin(115200, SERIAL_8N1, 25, 33);
  delay(1500);
}

//****************************************************************************************
// Write a message out to the Reyax LoRa module and wait for a reply.
String sendReceiveReyax(String s)
{

  String loraMsg;

  //debugUART.print("Sending: ");
  //debugUART.println(s);
  //Send the command
  LoRaUART.println(s);

  //Read reply. TODO: Parse Error codes. Add a timeout, etc.
  while (!LoRaUART.available())
  {
    delay(10);
  }

  loraMsg = LoRaUART.readStringUntil('\n');

  //debugUART.print("Received: ");
  //debugUART.println(loraMsg);
  return loraMsg;
}

//****************************************************************************************
// Set up the LoRa communication parameters from the config data structure
bool configureLoRa(Config &config)
{

  debugUART.println("  Configuring LoRa...");
  LoRaUART.println("AT");
  delay(1000);
  //sendReceiveReyax("AT"); // Get the module's attention
  //sendReceiveReyax("AT"); // Get the module's attention

  debugUART.println("    Setting Address to: " + config.loraAddress);
  ///config.loraAddress.trim();
  //debugUART.println(config.loraAddress);
  sendReceiveReyax("AT+ADDRESS=" + config.loraAddress);
  sendReceiveReyax("AT+ADDRESS?");

  debugUART.println("    Setting Network ID to: " + config.loraNetworkID);
  sendReceiveReyax("AT+NETWORKID=" + config.loraNetworkID);
  sendReceiveReyax("AT+NETWORKID?");

  debugUART.println("    Setting Band to: " + config.loraBand);
  sendReceiveReyax("AT+BAND=" + config.loraBand);
  sendReceiveReyax("AT+BAND?");

  debugUART.println("    Setting Modulation Parameters to: " + config.loraSF + "," + config.loraBW + "," + config.loraCR + "," + config.loraPreamble);
  sendReceiveReyax("AT+PARAMETER=" + config.loraSF + "," + config.loraBW + "," + config.loraCR + "," + config.loraPreamble);
  sendReceiveReyax("AT+PARAMETER?");

  //hwStatus+= "   LoRa : OK\n\n";
  return true;
}

//****************************************************************************************
// Sends a message to another LoRa module and listens for an ACK reply.
bool sendReceiveLoRa(String msg)
{
  long timeout = 2000; // TODO: Make this part of the Config struct -- better yet,
                       // calculate from the LoRa RF parameters and payload...
  bool replyPending = true;
  String strRetryCount;
  long t2, t1;

  t1 = millis();
  t2 = t1;

  strRetryCount = String(LoRaRetryCount);
  strRetryCount.trim();

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use https://arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<512> doc;
  char json[512] = {};

  // The message contains a JSON payload extract to the char array json
  msg.toCharArray(json, msg.length() + 1);

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, json);

  // Test if parsing succeeded.
  if (error)
  {
    debugUART.print(F("deserializeJson() failed: "));
    debugUART.println(error.f_str());
    return false;
  }

  doc["r"] = strRetryCount; // Add the retry count to the JSON doc

  msg = "";

  // Serialize JSON
  if (serializeJson(doc, msg) == 0)
  {
    Serial.println(F("Failed to write to string"));
    return false;
  }

  // Send the message. - Base stations use address 1.
  String reyaxMsg = "AT+SEND=1," + String(msg.length()) + "," + msg;

#if SHOW_DATA_STREAM
#else
  debugUART.print("Sending LoRa Message: ");
  debugUART.println(reyaxMsg);
#endif

  LoRaUART.println(reyaxMsg);

// Wait for ACK or timeout

// For Testing, don't wait for an ACK from a basestation
#if STAND_ALONE_LORA
  replyPending = false;
  return true;
#endif

  while ((replyPending == true) && ((t2 - t1) < timeout))
  {
    t2 = millis();
    if (LoRaUART.available())
    {
      String inString = LoRaUART.readStringUntil('\n');
      if (replyPending)
      {
        if (inString.indexOf("ACK") >= 0)
        {
          replyPending = false;
#if SHOW_DATA_STREAM
#else
          debugUART.println("ACK Received: " + inString);
#endif
          LoRaRetryCount = 0; // Reset for the next message.
          return true;
        }
      }
    }
  }

  if ((t2 - t1) >= timeout)
  {
#if SHOW_DATA_STREAM
#else
    debugUART.println("Timeout!");
    debugUART.println();
#endif
    LoRaRetryCount++;
    return false;
  }

  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

#endif // __DIGAME_LORA_H__