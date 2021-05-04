/*
 *  TFMini-plus LIDAR example program.
 * Copyright 2021, Digame Systems. All rights reserved.
 */
 
const int samples = 100;

//Falling Edge Model 100 pts
float fallingEdgeModel[] = {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float risingEdgeModel[] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};


float tinyFallingEdgeModel[] = {1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};


float notchModel[] =       {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};


float data[samples];
// 0.1 smooth of the falling edge model - 100 pts.
float smoothedFallingEdgeModel[]{
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
0.8,
0.62,
0.458,
0.3122,
0.18098,
0.062882,
-0.0434062,
-0.13906558,
-0.225159022,
-0.30264312,
-0.372378808,
-0.435140927,
-0.491626834,
-0.542464151,
-0.588217736,
-0.629395962,
-0.666456366,
-0.699810729,
-0.729829656,
-0.756846691,
-0.781162022,
-0.80304582,
-0.822741238,
-0.840467114,
-0.856420402,
-0.870778362,
-0.883700526,
-0.895330473,
-0.905797426,
-0.915217683,
-0.923695915,
-0.931326324,
-0.938193691,
-0.944374322,
-0.94993689,
-0.954943201,
-0.959448881,
-0.963503993,
-0.967153593,
-0.970438234,
-0.973394411,
-0.97605497,
-0.978449473,
-0.980604525,
-0.982544073,
-0.984289666,
-0.985860699,
-0.987274629,
-0.988547166,
-0.98969245 
};

// 0.1 smooth of the square wave model - 100 pts.
float model2[] = {
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
0.9,
0.81,
0.729,
0.6561,
0.59049,
0.531441,
0.4782969,
0.43046721,
0.387420489,
0.34867844,
0.313810596,
0.282429536,
0.254186583,
0.228767925,
0.205891132,
0.185302019,
0.166771817,
0.150094635,
0.135085172,
0.121576655,
0.109418989,
0.09847709,
0.088629381,
0.079766443,
0.071789799,
0.064610819,
0.058149737,
0.052334763,
0.047101287,
0.042391158,
0.038152042,
0.034336838,
0.030903154,
0.027812839,
0.025031555,
0.0225284,
0.02027556,
0.018248004,
0.116423203,
0.204780883,
0.284302795,
0.355872515,
0.420285264,
0.478256737,
0.530431064,
0.577387957,
0.619649161,
0.657684245,
0.691915821,
0.722724239,
0.750451815,
0.775406633,
0.79786597,
0.818079373,
0.836271436,
0.852644292,
0.867379863,
0.880641877,
0.892577689,
0.90331992,
0.912987928,
0.921689135,
0.929520222,
0.9365682,
0.94291138,
0.948620242,
0.953758217,
0.958382396,
0.962544156,
0.966289741,
0.969660766,
0.97269469,
0.975425221,
0.977882699,
0.980094429,
0.982084986,
0.983876487,
0.985488839,
0.986939955,
0.988245959,
0.989421363,
0.990479227,
0.991431304,
0.992288174,
0.993059357,
0.993753421,
0.994378079
};

float crossCorr[samples*4+1];



#include <CircularBuffer.h> // Adafruit library. Pretty small!
CircularBuffer<int, samples> buffer; // 


#include <TFMPlus.h>    // Include TFMini Plus LIDAR Library v1.4.0
TFMPlus tfmP;           // Create a TFMini Plus object

// Aliases for easier reading
#define debugUART Serial
#define tfMiniUART Serial2   

//****************************************************************************************
// Device initialization                                   
//****************************************************************************************
void setup()
{
    debugUART.begin(115200);   // Intialize terminal serial port
    delay(1000);               // Give port time to initalize

    pinMode(32,OUTPUT);

    /*
    debugUART.println("*****************************************************");
    debugUART.println("ParkData LIDAR Sensor Example");
    debugUART.println("Version 1.0");
    debugUART.println("Copyright 2021, Digame Systems. All rights reserved.");
    debugUART.println("*****************************************************");
    */
    tfMiniUART.begin(115200);  // Initialize TFMPLus device serial port.
    delay(1000);               // Give port time to initalize
    tfmP.begin(&tfMiniUART);   // Initialize device library object and...
                               // pass device serial port to the object.

    // Send some commands to the TFMini-Plus
    // - - Perform a system reset - -
    //debugUART.printf( "Activating LIDAR Sensor... ");
    if( tfmP.sendCommand(SYSTEM_RESET, 0)){
        //debugUART.println("Sensor Active.");
    }
    else tfmP.printReply();

    delay(1000);

    //debugUART.printf( "Adjusting Frame Rate... ");
    if( tfmP.sendCommand(SET_FRAME_RATE, FRAME_0)){
        //debugUART.println("Frame Rate Adjusted.");
    }
    else tfmP.printReply();

    
    //debugUART.println("Running!");  
    //delay(10000);

    //crossCorrelation(model1 , data2, samples, samples*2);
    

}

//****************************************************************************************
// Math functions
//****************************************************************************************

float mean(float a[], int numSamples){
    float result;
    
    result = 0;   
    for (int i=0; i<numSamples; i++) {
      result += a[i];
    }
    result /= numSamples;
    return result;  
}

// Calculate the correlation coefficient between two arrays
float correlation(float x[], float y[], int numSamples){ 
    float sx, sy, sxy, denom; 
    float mx, my; // means
    float r; // correlation coefficient

    /* Calculate the means */
    mx = mean(x, numSamples);
    my = mean(y, numSamples);
    
    /* Calculate the denominator */
    sx = 0;
    sy = 0;
    
    for (int i=0; i<numSamples; i++) {
      sx += (x[i] - mx) * (x[i] - mx);
      sy += (y[i] - my) * (y[i] - my);
    }
    
    denom = sqrt(sx*sy);

   /* Calculate the correlation coefficient */
    sxy = 0;
    for (int i=0; i<numSamples; i++) {
          sxy += (x[i] - mx) * (y[i] - my); 
    }
    r = sxy / denom;
    return r;
}


float crossCorrelation(float x[] , float  y[], int numSamples, int maxdelay){

   int i,j;
   float mx,my,sx,sy,sxy,denom,r;
   
   //debugUART.println("In crossCorrelation");
   /* Calculate the mean of the two series x[], y[] */
   //debugUART.println("Calculating Means");
   mx = 0;
   my = 0;   
   for (i=0;i<numSamples;i++) {
      mx += x[i];
      my += y[i];
   }
   mx /= numSamples;
   my /= numSamples;

   //debugUART.print("mx: ");
   //debugUART.print(mx);
   //debugUART.print("my: ");
   //debugUART.println(my);
   

   /* Calculate the denominator */
   //debugUART.println("Calculating Denominator");
   sx = 0;
   sy = 0;
   for (i=0;i<numSamples;i++) {
      sx += (x[i] - mx) * (x[i] - mx);
      sy += (y[i] - my) * (y[i] - my);
   }
   //debugUART.println("Dividing...");
   denom = sqrt(sx*sy);

   /* Calculate the correlation series */
   float rMax=0.0; //Initialize.
   float rMin=0.0; 
   int   rMaxIndex=0;

   debugUART.println("**********************");
   for (int myDelay=-maxdelay; myDelay<=maxdelay; myDelay++) {
      //debugUART.print(myDelay);
      //debugUART.println(" ");
      sxy = 0;
      for (i=0;i<numSamples;i++) {
         j = i + myDelay;
         
         if (j < 0 || j >= numSamples)
            continue;
         else
            sxy += (x[i] - mx) * (y[j] - my); 
      }
      //debugUART.print(denom);
      //debugUART.print(" ");
      
      r = sxy / denom;
      if (r>rMax){
        rMax=r; rMaxIndex = myDelay + maxdelay;
      }
      
      if (r<rMin){rMin=r;}
      
      //debugUART.print(myDelay+maxdelay);
      //debugUART.print(" ");
      //debugUART.println(r*100);
      crossCorr[myDelay+maxdelay]=r;

      //debugUART.print(myDelay+maxdelay);
      //debugUART.print(" ");
      //debugUART.println(r);
      /* r is the correlation coefficient at "delay" */

   }
   
   
   //return rMax*100;
   
   return 400*(rMin+rMax)/2;

   if (abs(rMin)>=rMax){
     return rMin*100;
   } else {
     return rMax*100;
   }
 
}


// Initialize some variables
int16_t tfDist = 0;    // Distance to object in centimeters
int16_t tfFlux = 0;    // Strength or quality of return signal
int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
float   smoothed = 0.0;
float correl1 = 0.0;
float correl2 = 0.0;

int lidarUpdateRate = 8; // 100Hz -> 10 ms
int slowUpdateRate = 100;
int fastUpdateRate = 10;

bool carIsPresent = false;
bool carWasPresent = false;
int32_t carDiscoveredTime = 0;

int16_t highThreshold = 30;
int16_t lowThreshold = -30; 

int16_t highEventValue = highThreshold;
int16_t lowEventValue  = lowThreshold;

int carEventValue = 500;

void blink(){
  digitalWrite(32, HIGH);
  delay(10);
  digitalWrite(32,LOW);
}


//************************************************************************
void loop(){

    tfmP.sendCommand(TRIGGER_DETECTION, 0);
    //delay(lidarUpdateRate);
    
    // Read the LIDAR Sensor
    if( tfmP.getData(tfDist, tfFlux, tfTemp) ) { 
      tfDist = tfDist + random(0,0); // +/- cm random noise...
     
      buffer.push(tfDist);
      
      for (byte i = 0; i < buffer.size(); i++) {
        data[i]=buffer[i];
      }
      
      //correl1 = crossCorrelation(fallingEdgeModel, data, samples, samples*2) *2 ;
      data[0]=1;

      correl1 = (correlation(fallingEdgeModel, data, samples)* 100)-10;//-83.0;
      
      //Filter the correlated value
      smoothed = smoothed * 0.8 + (float)correl1 * 0.2;
           
      debugUART.print(tfDist+300);
      debugUART.print(" ");

      debugUART.print(smoothed);
      debugUART.print(" ");

      if (smoothed < lowThreshold){
        lowEventValue = lowThreshold + 100;   
        digitalWrite(32, HIGH);
      } else {
        lowEventValue = lowThreshold;  
        digitalWrite(32, LOW);
      }

      if (smoothed>  highThreshold){
        highEventValue = highThreshold + 100;   
      } else {
        highEventValue = highThreshold;  
      }

      debugUART.print(lowEventValue);
      debugUART.print(" ");
      debugUART.print(highEventValue);
      debugUART.print(" ");

      
      debugUART.print(100);
      debugUART.print(" ");
      debugUART.print(-100);
      debugUART.println();
    }

}
