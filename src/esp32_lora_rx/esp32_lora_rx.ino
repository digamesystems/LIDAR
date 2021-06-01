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


void loop() {
  
  if (loraUART.available()) {
    String inString = loraUART.readStringUntil('\n');
    if (inString.indexOf("+RCV")>=0){
      
      debugUART.print("I heard: ");   
      loraUART.println("AT+SEND=1,3,ACK");
      digitalWrite(RX_LED, HIGH);   
      delay(100);
      digitalWrite(RX_LED, LOW);  
      
    } 
    debugUART.println(inString);      
  }

  if (debugUART.available()) {
    int inByte = Serial.read();
    loraUART.write(inByte);
  }

}
