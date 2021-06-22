/* Send repeated LoRa messages to an address. 
*/

#define loraUART Serial1
#define debugUART Serial

int TX_LED = 13;

void setup() {

  pinMode(TX_LED, OUTPUT);

  loraUART.begin(115200, SERIAL_8N1, 25,33);
  debugUART.begin(115200);
  delay(1000);
  
  debugUART.println();
  debugUART.println("**********************");
  debugUART.println("TX (TRANSMIT) MODULE ");
  debugUART.println("Version 0.9.2 ");
  debugUART.println("**********************");
  
  //debugUART.println("Setting Network ID");
  //loraUART.println("AT+NETWORKID=7");
  //delay(100);
  debugUART.println("Setting Device Address");
  loraUART.println("AT+ADDRESS=6"); //MY ADDRESS
  delay(1000);
  //debugUART.println("Setting RF Params");
  //loraUART.println("AT+PARAMETER=7,9,1,7");
  //delay(100);
  
}

// Sends a message to another LoRa module and listens for an ACK reply.
bool sendReceiveLoRa(String msg){
  long timeout = 3000;
  long t2,t1;

  t1 = millis();
  t2 = t1;

  bool replyPending = true;
  
  //Send the message... Base Stations Default to an address of 1.
  String reyaxMsg = "AT+SEND=1,"+String(msg.length())+","+msg;
  debugUART.print("Sending LoRa Message: ");
  debugUART.println(reyaxMsg);
  loraUART.println(reyaxMsg);
  
  //wait for ACK or timeout
  while ((replyPending == true) && ((t2-t1)<timeout)){
    t2=millis();
    if (loraUART.available()) {
      String inString = loraUART.readStringUntil('\n');
      if (replyPending) {
        if (inString.indexOf("ACK")>=0){
          replyPending = false;
          debugUART.println("ACK Received: " + inString);

          // Blinky!
          digitalWrite(TX_LED, HIGH);   
          delay(100);
          digitalWrite(TX_LED, LOW);  

          return true;
          
        }
      } 
    } 
  }
  
  if((t2-t1)>= timeout){
    debugUART.println("Timeout!");
    return false;
  }
  
}


long count = 0; 
bool txActive = true; 

void loop() {

  if (loraUART.available()) {
    String inString = loraUART.readStringUntil('\n');
    debugUART.println(inString);
  }

  if (debugUART.available()) {
    String inString = debugUART.readStringUntil('\n');
    inString.trim();
    
    if (!txActive){
        loraUART.println(inString);  
    }
    if (inString.indexOf("PAUSE")>=0) {
      debugUART.println("Pausing TX...");
      txActive = false;
    }
    if (inString.indexOf("GO")>=0) {
      debugUART.println("Resuming TX...");
      txActive = true;
    } 
  }

  if (txActive){
    String strCount(count);
    debugUART.println(strCount);
    while (!sendReceiveLoRa(strCount)){}
    //delay(2000);
    count++;
  }


}
