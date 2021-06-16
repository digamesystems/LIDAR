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

  //debugUART.println("Setting Network ID");
  //loraUART.println("AT+NETWORKID=7");
  //delay(100);
  //debugUART.println("Setting Device Address");
  //loraUART.println("AT+ADDRESS=1"); //MY ADDRESS
  //delay(100);
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
  
  //Send the message... 
  loraUART.println("AT+SEND=2,"+String(msg.length())+","+msg);
  
  //wait for ACK or timeout
  while ((replyPending == true) && ((t2-t1)<timeout)){
    t2=millis();
    if (loraUART.available()) {
      String inString = loraUART.readStringUntil('\n');
      if (replyPending) {
        if (inString.indexOf("ACK")>=0){
          replyPending = false;
          debugUART.println(inString);
          
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

void loop() {

  // read from port 1, send to port 0:
/*
  if (loraUART.available()) {
    String inString = loraUART.readStringUntil('\n');
    debugUART.println(inString);
  }

  // read from port 0, send to port 1:
  if (debugUART.available()) {
    int inByte = debugUART.read();
    loraUART.write(inByte);
  }

*/

  String strCount(count);
  debugUART.println(strCount);
  while (!sendReceiveLoRa(strCount)){}
  //loraUART.println("AT+SEND=2,"+String(strCount.length())+","+strCount);
  delay(900);
  count++;

  
}
