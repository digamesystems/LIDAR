/* Listen for lora messages.
 */

#define loraUART Serial1
#define debugUART Serial

int RX_LED = 13;

void setup() {

  pinMode(RX_LED, OUTPUT);
  
  loraUART.begin(115200, SERIAL_8N1, 25,33);
  debugUART.begin(115200);
  delay(1000);
  
  debugUART.println();
  debugUART.println("**********************");
  debugUART.println("RX (RECEIVER) MODULE: ");
  debugUART.println("**********************");
  
  //debugUART.println("Setting Network ID");
  //loraUART.println("AT+NETWORKID=7");
  //delay(100);
  //debugUART.println("Setting Device Address");
  //loraUART.println("AT+ADDRESS=2"); //MY ADDRESS
  //delay(100);
  //debugUART.println("Setting RF Params");
  //loraUART.println("AT+PARAMETER=7,9,1,7");
  //delay(100);
}

String inString;
void loop() {
  
  if (loraUART.available()) {
    inString = loraUART.readStringUntil('\n');
    if (inString.indexOf("+RCV")>=0){
      
      debugUART.print("Received LoRa Message: ");
      debugUART.println(inString);

      debugUART.print("Sending ACK: ");
      loraUART.println("AT+SEND=2,3,ACK");
      
      digitalWrite(RX_LED, HIGH);   
      delay(100);
      digitalWrite(RX_LED, LOW);  
      
    } else {
      debugUART.println(inString);
    }      
  }

  if (debugUART.available()) {
    inString = debugUART.readStringUntil('\n');
    loraUART.println(inString);
  }

}
