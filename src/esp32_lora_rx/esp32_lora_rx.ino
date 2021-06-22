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
  debugUART.println("RX (RECEIVER) MODULE ");
  debugUART.println("Verson 0.9.2");
  debugUART.println("**********************");
  
  //debugUART.println("Setting Network ID");
  //loraUART.println("AT+NETWORKID=7");
  //delay(100);
  debugUART.println("Setting Device Address");
  loraUART.println("AT+ADDRESS=1"); //MY ADDRESS -- Base Stations Default to 1.
  delay(1000);
  //debugUART.println("Setting RF Params");
  //loraUART.println("AT+PARAMETER=7,9,1,7");
  //delay(100);
}

String msg;
void loop() {
  
  if (loraUART.available()) {
    msg = loraUART.readStringUntil('\n');
    if (msg.indexOf("+RCV")>=0){
      
      debugUART.print("Received LoRa Message: ");
      debugUART.println(msg);

      //Grab the address of the sender
      // Messages are of the form: "+RCV=2,2,16,-64,36" -- where the first number after the "=" is the address.
      
      // Start and end of the JSON payload in the msg.
      int idxstart = msg.indexOf('=')+1;
      int idxstop = msg.indexOf(',');
 
      // grab the address
      String senderAddress = msg.substring(idxstart,idxstop); 

      // Now we can answer anyone who talks to us.
      debugUART.print("Sending ACK: ");
      loraUART.println("AT+SEND=" + senderAddress + ",3,ACK"); 

      //Blinky!
      digitalWrite(RX_LED, HIGH);   
      delay(100);
      digitalWrite(RX_LED, LOW);  
      
    } else {
      debugUART.println(msg);
    }      
  }

  if (debugUART.available()) {
    msg = debugUART.readStringUntil('\n');
    msg.trim();
    loraUART.println(msg);
  }

}
