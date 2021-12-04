/*
 *  TFMini-plus LIDAR example program.
 * Copyright 2021, Digame Systems. All rights reserved.
 */


#define tfMiniUART_1 Serial1
#define tfMiniUART_2 Serial2

#include <TFMPlus.h>    // Include TFMini Plus LIDAR Library v1.4.0

TFMPlus tfmP_1;           // Create a TFMini Plus object
TFMPlus tfmP_2;           // Create a TFMini Plus object


//****************************************************************************************
void init_TFMPlus(TFMPlus &tfmP, int port=1){
   
  // Initialize device library object and pass device serial port to the object.
  if (port == 1){
    tfmP.begin(&tfMiniUART_1);   
  }else if (port == 2){
    tfmP.begin(&tfMiniUART_2);   
  }else{
    Serial.println("Unknown Port.");
    return;
  }
     
  // Send some commands to configure the TFMini-Plus
  
  // Perform a system reset
  Serial.printf( "Activating LIDAR Sensor... ");
  if( tfmP.sendCommand(SYSTEM_RESET, 0)){
      Serial.println("Sensor Active.");
  }
  else{
    Serial.println("TROUBLE ACTIVATING LIDAR!");                    
    tfmP.printReply();
  }

  delay(1000);

  // Set the acquisition rate to 100 Hz.
  Serial.printf( "Adjusting Frame Rate... ");
  if( tfmP.sendCommand(SET_FRAME_RATE, FRAME_100)){
      Serial.println("Frame Rate Adjusted.");
  }
  else tfmP.printReply();
 
}

//****************************************************************************************
int process_LIDAR(TFMPlus &tfmP, float &smoothed, int offset){
  int16_t tfDist = 0;    // Distance to object in centimeters
  int16_t tfFlux = 0;    // Strength or quality of return signal
  int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
  
  int lidarUpdateRate = 10; // 100Hz -> 10 ms
  int targetVisible = false;
  
  tfmP.sendCommand(TRIGGER_DETECTION, 0);
  //delay(lidarUpdateRate);

  // Read the LIDAR Sensor
  if( tfmP.getData( tfDist, tfFlux, tfTemp)) { 

    //Filter the measured distance
    smoothed = smoothed * 0.95 + (float)tfDist * 0.05;
    
    //Serial.print(tfDist + offset);
    //Serial.print(" ");
    
    
    Serial.print(smoothed);
    Serial.print(" ");

    targetVisible = (smoothed < 160);

    if (targetVisible){
      Serial.print(300 + offset);
    } else {
      Serial.print(200 + offset);
    }  
    Serial.print(" ");

    if (targetVisible) {
      return 1;
    } else {
      return 0;
    }
   
  }

  return -1; 
    
}

//****************************************************************************************
// Device initialization                                   
//****************************************************************************************
void setup()
{
    Serial.begin(115200);   // Intialize terminal serial port
    delay(1000);               // Give port time to initalize
    
    Serial.println("*****************************************************");
    Serial.println("ParkData LIDAR Sensor Example");
    Serial.println("Version 1.0");
    Serial.println("Copyright 2021, Digame Systems. All rights reserved.");
    Serial.println("*****************************************************");

    tfMiniUART_1.begin(115200);//,SERIAL_8N1,25,33);  // Initialize TFMPLus device serial port.
    tfMiniUART_2.begin(115200,SERIAL_8N1,25,33);  // Initialize TFMPLus device serial port.

    init_TFMPlus(tfmP_1, 1);
    init_TFMPlus(tfmP_2, 2);
    
}


//****************************************************************************************
// Smoothing the LIDAR signals to get rid of noise and introduce a bit 
// of a decay time. 
  float smoothed_LIDAR_1 = 0.0;
  float smoothed_LIDAR_2 = 0.0;

// We're tracking the visibility of the target on both sensors. Valid 
// events go from only visible on one sensor to being visible on both. 
// Direction is determined by which sensor sees the target first. 
  int previousState = 0; 
  int state = 0;  

//****************************************************************************************
// Main Loop                                   
//****************************************************************************************
void loop(){
  int value = 0;
  state = 0;
  value = process_LIDAR(tfmP_1, smoothed_LIDAR_1, 100); 
  if (value>=0){
      state = state + value;
      value = process_LIDAR(tfmP_2, smoothed_LIDAR_2, 110);
      state = state + value * 2;
      
      Serial.print(" ");
      //Serial.print(state);
      Serial.println();

      if ((previousState == 2)&&(state ==3)){
        Serial.println("IN_Event!");  
      }

      if ((previousState == 1) && (state == 3)){
        Serial.println("OUT_Event!");  
      }
      
      previousState = state;
  }
}
