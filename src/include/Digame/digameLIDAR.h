#ifndef __DIGAME_LIDAR_H__
#define __DIGAME_LIDAR_H__

#define debugUART Serial
#define tfMiniUART Serial2

#define LOG_RAW_DATA_TO_SD false // Log the Raw data to the SD Card for analysis.


#include <digameJSONConfig.h> // Program parameters from config file on SD card
#include <digamePowerMgt.h>
#include <TFMPlus.h>          // Include TFMini Plus LIDAR Library v1.4.0
TFMPlus tfmP;                 // Create a TFMini Plus object

#include <CircularBuffer.h> // Adafruit library. Pretty small!

int16_t initLIDARDist = 999; // The initial distance measured by the lidar when it wakes up.

const int lidarSamples = 25;
String lastDistanceMeasured = "0";

CircularBuffer<int, lidarSamples> lidarBuffer; // We're going to hang onto the last 100 raw data
                                               //   points to visualize what the sensor sees

CircularBuffer<int, 150> lidarHistoryBuffer; // A longer buffer for visualization of the history
                                             // before the algorithm makes a decision.

const int histogramSize = 121; // Playing with a histogram of distances to see if we can learn
                               //   how to determine lane posistions on our own. 10 cm bins
unsigned long lidarDistanceHistogram[histogramSize];

unsigned long lidarTimeHistogram[histogramSize]; // 0-12 seconds in tenth of a second bins
String logFileName;        // Log data to a file of this name when LOG_RAW_DATA_TO_SD is true.
File logFile;


bool initLIDAR(bool);
void showLIDARDistanceHistogram();
void clearLIDARDistanceHistogram();
String getDistanceHistogramString();









//*****************************************************************************
// Set up the LIDAR sensor in triggered mode
bool initLIDAR(bool triggeredMode = false)
{

  tfMiniUART.begin(115200); // Initialize TFMPLus device serial port.
  delay(1000);              // Give port time to initalize
  tfmP.begin(&tfMiniUART);  // Initialize device library object and...
                            // pass device serial port to the object.

  // Perform a system reset
  debugUART.print("  Activating LIDAR Sensor... ");

  if (tfmP.sendCommand(SOFT_RESET, 0))
  {

    debugUART.println("Done. (LIDAR Sensor initialized)");
    //hwStatus+="   LIDAR: OK\n\n";
    delay(500);
    if (triggeredMode)
    {
      debugUART.print("    Adjusting LIDAR Frame Rate... ");
      if (tfmP.sendCommand(SET_FRAME_RATE, FRAME_0))
      { //FRAME_0 is triggered mode.
        debugUART.println("Frame Rate Adjusted.");
      }
    }

    //GRAB an initial Value to test: 
    debugUART.print("    Initial LIDAR Reading: ");
    bool result; 
    int16_t dist;
    
    tfmP.sendCommand(TRIGGER_DETECTION, 0); // Trigger a LIDAR measurment
    delay(100);                 // Wait a bit...
    result = tfmP.getData(dist); // Grab the results

    
    
    if ((result) || (tfmP.status == TFMP_WEAK)) // Process good measurements and treat weak ones as 'infinity'
    {
      // When very close, or looking off into empty space, the sensor reports zero or a negative value.
      // The short range isn't an issue for us.
      // Any Zeros we see will be due to no reflective target in range.
      if ((dist < 0) || (dist >= 1000)) // Added the second check in case we pick up a long-range target. 
      {
        dist = 999;
      }
      initLIDARDist = dist;
      debugUART.println(dist);
    }

    
    return true;
  }
  else
  {

    debugUART.println("ERROR! LIDAR Sensor not found or sensor error.");
    //hwStatus+="   LIDAR: ERROR!\n\n";

    return false;
  }
}

//*****************************************************************************
void showLIDARDistanceHistogram()
{
  debugUART.println(getDistanceHistogramString());
}

//*****************************************************************************
void clearLIDARDistanceHistogram()
{
  for (int i = 0; i < histogramSize; i++)
  {
    lidarDistanceHistogram[i] = 0;
  }
}

//*****************************************************************************
// return the histogram as a table
String getDistanceHistogramString()
{
  String retValue = "D (cm), Counts\n";
  for (int i = 0; i < histogramSize; i++)
  {
    retValue = retValue + String(i * 10) + ", " + String(lidarDistanceHistogram[i]) + "\n";
  }
  return retValue;
}



String getDistanceHistogramChartString(Config config)
{
  String retValue =     "LIDAR DISTANCE HISTOGRAM \nD (cm)\t|  Counts\n";
  retValue = retValue + "--------------------------\n";

  unsigned long minValue = 0;
  unsigned long maxValue = 0;
  int maxIndex = 0;

  if ((config.lidarZone2Max.toInt()/10) < histogramSize){
    maxIndex = config.lidarZone2Max.toInt()/10;
  } else {
    maxIndex = histogramSize;
  }
  
  for (int i = 0; i < maxIndex; i++){
    if (lidarDistanceHistogram[i] < minValue) minValue = lidarDistanceHistogram[i];
    if (lidarDistanceHistogram[i] > maxValue) maxValue = lidarDistanceHistogram[i];
  }

  if (maxValue > 0){
    for (int i = 0; i < maxIndex; i++){
      if ((i % 5 )==0){  //Put a tic on the axis every 50 cm.
        retValue = retValue + String (i * 10) + "\t+";
      }else{
        retValue = retValue + String (i * 10) + "\t|"; 
      }

      // Our charting routine. Welcome back to 1972!
      for (int j = 0; j < ((100 * lidarDistanceHistogram[i]) / maxValue); j++){
        retValue = retValue + "*"; 
      }  
      retValue = retValue + "\n";

    }
   
   return retValue;
    
  } else {

    return "No data, yet.";
  }


}


//*****************************************************************************
// A pretty simple zone-based scheme for detecting vehicles. A vehicle must be 
// in a zone for a period of time to count as 'present'. When it leaves the zone
// an event is generated. 

int processLIDARSignal(Config config)
{
  // LIDAR signal analysis parameters

  int16_t tfDist = 0;          // Distance to object in centimeters
  int16_t tfFlux = 0;          // Strength or quality of return signal
  int16_t tfTemp = 0;          // Internal temperature of Lidar sensor chip
  static float smoothed = 0.0; // A smoothed version of the raw distance data

  static bool carPresentLane1 = false;         // Do we see a car now?
  static bool previousCarPresentLane1 = false; // Had we seen a car last time?

  static bool carPresentLane2 = false;         // Do we see a car now?
  static bool previousCarPresentLane2 = false; // Had we seen a car last time?

  static unsigned long firstInRangeMS1 = 0; // The time in ms, the car first got
  static unsigned long firstInRangeMS2 = 0; // close enough to count as 'present'

  static unsigned long timeInRange1 = 0; // How long has the car close enough to be 'present'
  static unsigned long timeInRange2 = 0;

  unsigned int carEvent1 = 0;                                                       // A variable for the serial plotter.
  unsigned int carEvent2 = 0;                                                       // A variable for the serial plotter.
  
  unsigned int lidarUpdateRate = 15;                                               // Time in ms between readings
  unsigned long minTimeInRange = (unsigned long)config.lidarResidenceTime.toInt(); // Minimum time to count as fully 'present'.

  int retValue = 0; // Return value. Do we have a vehicle event?
  bool lidarResult = false;

  tfmP.sendCommand(TRIGGER_DETECTION, 0); // Trigger a LIDAR measurment
  delay(lidarUpdateRate);                 //

  lidarResult = tfmP.getData(tfDist, tfFlux, tfTemp);
  /*
    debugUART.print(lidarResult);
    debugUART.print(" ");
    debugUART.print(tfmP.status);
    debugUART.print(" ");
    debugUART.println(tfDist);
  */

  // Read the LIDAR Sensor
  if ((lidarResult) || (tfmP.status == TFMP_WEAK)) // Process good measurements and treat weak ones as 'infinity'
  {
    // When very close, or looking off into empty space, the sensor reports zero or a negative value.
    // The short range isn't an issue for us.
    // Any Zeros we see will be due to no reflective target in range.
    // Assume max range for the sensor to get the threshold algorithm to perform properly.
    if (tfDist <= 0)
    {
      tfDist = 1200;
    }

    //Filter the measured distance
    smoothed = smoothed * (config.lidarSmoothingFactor.toFloat()) +
               (float)tfDist * (1.0 - config.lidarSmoothingFactor.toFloat());

    int intSmoothed = (int)smoothed * 10;

    lidarDistanceHistogram[(unsigned int)(tfDist / 10)]++; // Grabbing a histogram of distances
                                                           // to explore automatic lane determination...

    lidarBuffer.push(intSmoothed); // Keep the last 100 points of smoothed data history for analysis
    

    if ((smoothed < config.lidarZone1Max.toFloat()) &&
        (smoothed > config.lidarZone1Min.toFloat()))
    {
      timeInRange1 = millis() - firstInRangeMS1; // How long has our visitor been in range?

      if (timeInRange1 > minTimeInRange)
      { // Is that long enough to count as present?
        carPresentLane1 = true;
      }
    }
    else
    { // No one is close enough to count as present.
      timeInRange1 = 0;
      firstInRangeMS1 = millis();
      carPresentLane1 = false;
    }

    if ((smoothed < config.lidarZone2Max.toFloat()) &&
        (smoothed > config.lidarZone2Min.toFloat()))
    {
      timeInRange2 = millis() - firstInRangeMS2; // How long has our visitor been in range?

      if (timeInRange2 > minTimeInRange)
      { // Is that long enough to count as present?
        carPresentLane2 = true;
      }
    }
    else
    { // No one is close enough to count as present.
      timeInRange2 = 0;
      firstInRangeMS2 = millis();
      carPresentLane2 = false;
    }

    if ((previousCarPresentLane1 == true) && (carPresentLane1 == false)) 
    { // The car has left the field of view.
      carEvent1 = 300;
      retValue = 1;
    }
    else
    {
      carEvent1 = 0;
    }

    if ((previousCarPresentLane2 == true) && (carPresentLane2 == false))    
    { // The car has left the field of view.
      carEvent2 = 300;
      retValue = 2;
    }
    else
    {
      carEvent2 = 0;
    }

if (config.showDataStream == "true"){
    debugUART.print(tfDist);
    debugUART.print(",");
    debugUART.print(smoothed);
    debugUART.print(",");
    debugUART.print(config.lidarZone1Max.toFloat());
    debugUART.print(",");
    debugUART.print(config.lidarZone1Min.toFloat());
    debugUART.print(",");
    debugUART.print(config.lidarZone2Max.toFloat());
    debugUART.print(",");
    debugUART.print(config.lidarZone2Min.toFloat());
    debugUART.print(",");
    debugUART.print(carEvent1);
    debugUART.print(",");
    debugUART.println(carEvent2);
}

    previousCarPresentLane1 = carPresentLane1;
    previousCarPresentLane2 = carPresentLane2;
  }
  else
  {

    // Report the error
    // tfmP.printStatus();
  }

  return retValue;
}

void logToSDCard(){
    #if LOG_RAW_DATA_TO_SD

      if (!logFile)
        {
          debugUART.println(F("    Failed to create file!"));
        } else {
          //debugUART.println("    Writing file...");
          logFile.print(String(millis()));
          logFile.print(",");
          logFile.println(String(tfDist));
          // Close the file
          //debugUART.println(String(millis())+","+String(tfDist));
          logFile.flush(); // Opening the file in setup and flushing for speed. 
        }
    #endif


}

/****************************************************************************************
 24 Sept 2021 
 Trying a new approach. This routine uses a 'voting' scheme to disposition if a vehicle 
 has been in a lane in the last n points. Since we have already been using a circular
 buffer to capture a little history, let's try using that history rather than a smoothed
 data value. 
 
 This might be better for some kinds of noisy data like we're seeing from black cars.
 ****************************************************************************************/
int processLIDARSignal2(Config config){
  // LIDAR signal analysis parameters

  int16_t tfDist = 0;          // Distance to object in centimeters
  int16_t tfFlux = 0;          // Strength or quality of return signal
  int16_t tfTemp = 0;          // Internal temperature of Lidar sensor chip
  
  static bool carPresentLane1 = false;         // Do we see a car now?
  static bool previousCarPresentLane1 = false; // Had we seen a car last time?

  static bool carPresentLane2 = false;         // Do we see a car now?
  static bool previousCarPresentLane2 = false; // Had we seen a car last time?

  unsigned int carEvent1 = 0;                  // A variable for the serial plotter.
  unsigned int carEvent2 = 0;                  // A variable for the serial plotter.
  
  unsigned int lidarUpdateRate = 10;           // Time in ms between readings
  
  int retValue = 0;          // Return value. Do we have a vehicle event?
  bool lidarResult = false;  // return value from reading the LIDAR sensor

  tfmP.sendCommand(TRIGGER_DETECTION, 0); // Trigger a LIDAR measurment
  delay(lidarUpdateRate);                 // Wait a bit...

  lidarResult = tfmP.getData(tfDist, tfFlux, tfTemp); // Grab the results


  if ( (lidarResult) || (tfmP.status == TFMP_WEAK) ) // Process good measurements 
                                                   // or weak ones 
  {
    // Check the status code if not "ready" one of several errors has occured.
    // Looking at the source in TFMPlus.cpp, lidarResult should only be true
    // if everything is ok. Processing weak signals to avoid lockup looking off
    // into infinity. 

    if (tfmP.status !=TFMP_READY) {tfDist = 1001;} // "something" is weird.

    // When very close, or looking off into empty space, the sensor reports zero or a negative value.
    // The short range isn't an issue for us.
    // Any Zeros we see will be due to no reflective target in range.
    if ((tfDist <= 0) || (tfDist >= 1000)) // Added the second check in case we pick up a long-range target. 
    {
      tfDist = 999;
    }
    
    lastDistanceMeasured = String(tfDist);

    lidarDistanceHistogram[(unsigned int)(tfDist / 10)]++; // Grabbing a histogram of distances
                                                           // to explore automatic lane determination...
                                                           // The check for >= 1000 above is to avoid
                                                           // exceeding the limits of the histogram array.

    lidarBuffer.push(tfDist); // The circular buffer of LIDAR data for analysis
    lidarHistoryBuffer.push((tfDist)); // A longer history for display.

    long zone1Strength = 0;  // A measure of how 'present' a car is in each lane over an interval of time
    long zone2Strength = 0;

    for (int i = 0; i < lidarBuffer.size(); i++)
    {
      if ((lidarBuffer[i] < config.lidarZone1Max.toInt()) &&
          (lidarBuffer[i] > config.lidarZone1Min.toInt()))
      {
        zone1Strength = zone1Strength + 1; // Add 'car-ness' to Zone 1
      }

      if ((lidarBuffer[i] < config.lidarZone2Max.toInt()) &&
          (lidarBuffer[i] > config.lidarZone2Min.toInt()))
      {
        zone2Strength = zone2Strength + 1; // Add 'car-ness' to Zone 2       
      }
        
    }

    // Normalize to 100%
    zone1Strength = (100 * zone1Strength / lidarBuffer.size());
    zone2Strength = (100 * zone2Strength / lidarBuffer.size());

    int threshold = config.lidarResidenceTime.toInt(); // Minimum 'car-ness' to count as 'car' TODO: Make tweakable

    previousCarPresentLane1 = carPresentLane1;
    previousCarPresentLane2 = carPresentLane2;

    carPresentLane1 = (zone1Strength > threshold);
    carPresentLane2 = (zone2Strength > threshold);

    if ((previousCarPresentLane1 == true) && (carPresentLane1 == false)) 
    { // The car was here and now has left the field of view.
      carEvent1 = 300;
      retValue = 1;
    }
    else
    {
      carEvent1 = 0;
    }

    if ((previousCarPresentLane2 == true) && (carPresentLane2 == false))    
    { // The car was here and now has left the field of view.
      carEvent2 = 300;
      retValue = 2;
    }
    else
    {
      carEvent2 = 0;
    }

// For the serial plotter.
     if (config.showDataStream == "true"){
        debugUART.print(tfDist);
        debugUART.print(",");
        debugUART.print(config.lidarZone1Max.toFloat());
        debugUART.print(",");
        debugUART.print(config.lidarZone1Min.toFloat());
        debugUART.print(",");
        //debugUART.print(config.lidarZone2Max.toFloat());
        //debugUART.print(",");
        //debugUART.print(config.lidarZone2Min.toFloat());
        //debugUART.print(",");
        debugUART.print(carEvent1);
        debugUART.print(",");
        debugUART.print(carEvent2);
        debugUART.print(",");
        debugUART.print(zone1Strength);
        debugUART.print(",");
        debugUART.println(zone2Strength);
     }

  }
  else
  {
    // Report the error
    // tfmP.printStatus();
    // Other than TFMP_WEAK, the other error I see occassionaly is 'CHECKSUM'
    // Why would serial com have a problem?...
    // TODO: Investigate.
  }

  return retValue;

}

/****************************************************************************************
22 Nov 2021 
 This is a variation on processLIDARSignal2. 

 Here, we're trying to mitigate false counts due to shadowing by closer vehicles.  
 zoneStrength gets added when a car is present in the lane. 
 
 It gets subtracted when the signal is farther than the outer limit of the lane. 
 
 Signals closer than the lower limit of the lane are ignored so cars in closer lanes 
 don't generate false events in farther lanes as they pass. 
 
 ****************************************************************************************/
int processLIDARSignal3(Config config){
  // LIDAR signal analysis parameters

  int16_t tfDist = 0;          // Distance to object in centimeters
  int16_t tfFlux = 0;          // Strength or quality of return signal
  int16_t tfTemp = 0;          // Internal temperature of Lidar sensor chip
  
  static float zone1Strength = 0;  // A measure of how 'present' a car is in each lane over an interval of time
  static float zone2Strength = 0;  // Now static and doesn't reset on every call

  static bool carPresentLane1 = false;         // Do we see a car now?
  static bool previousCarPresentLane1 = false; // Had we seen a car last time?

  static bool carPresentLane2 = false;         // Do we see a car now?
  static bool previousCarPresentLane2 = false; // Had we seen a car last time?

  unsigned int carEvent1 = 0;                  // A variable for the serial plotter.
  unsigned int carEvent2 = 0;                  // A variable for the serial plotter.
  
  unsigned int lidarUpdateRate = 10;           // Time in ms between readings
  
  int retValue = 0;          // Return value for the routine. Do we have a vehicle event? 
                             //  Which lane?

  bool lidarResult = false;  // Return value from reading the LIDAR sensor

  int threshold = config.lidarResidenceTime.toInt(); // Level of signal to count as present.

// Trying an experiment. Let the LIDAR run free and poll it occassionally.
  //tfmP.sendCommand(TRIGGER_DETECTION, 0); // Trigger a LIDAR measurment
  //delay(lidarUpdateRate);                 // Wait a bit...
  //lightSleepMSec(lidarUpdateRate);

  lidarResult = tfmP.getData(tfDist, tfFlux, tfTemp); // Grab the results

  if ( (lidarResult) || (tfmP.status == TFMP_WEAK) ) // Process good measurements 
                                                     // or weak ones 
  {
    // Check the status code. If not "ready" one of several errors has occurred.
    // Looking at the source in TFMPlus.cpp, lidarResult should only be true
    // if everything is OK. Processing weak signals to avoid lockup looking off
    // into infinity. 

    if (tfmP.status !=TFMP_READY) {tfDist = 1001;} // "something" is weird.

    // When very close, or looking off into empty space, the sensor reports zero or a 
    // negative value. The short range isn't an issue for us.
    // Any Zeros we see will be due to no reflective target in range.
    if ((tfDist <= 0) || (tfDist >= 1000)) // Added the second check in case we pick up 
                                           // a long-range target. 
    {
      tfDist = 999; // Limiting to 999 saves a digit in the messages to the server.
    }
    
    lastDistanceMeasured = String(tfDist);

    lidarDistanceHistogram[(unsigned int)(tfDist / 10)]++; // Grabbing a histogram of distances
                                                           // to explore automatic lane determination...
                                                           // The check for >= 1000 above is to avoid
                                                           // exceeding the limits of the histogram array.

    lidarBuffer.push(tfDist); // The circular buffer of LIDAR data for analysis
    lidarHistoryBuffer.push((tfDist)); // A longer history for display.

    // TODO: Trying out a pre-filter here to look at the lidar history buffer 
    // and do a disposition of whether we should take this data seriously...
    // There is a difference between 'glimpsing' something and 'seeing' it. I'm 
    // thinking of how snow can cause short little events around 1-2 meters...

    // PRE-FILTER: Do we have enough signal to count as car-ness?
    float bufferInteg1 = 0; // Integral of in-zone data in the buffer
    float bufferInteg2 = 0;
    
    for (int i = 0; i < lidarBuffer.size(); i++)
    {
      if ((lidarBuffer[i] < config.lidarZone1Max.toInt()) &&
          (lidarBuffer[i] > config.lidarZone1Min.toInt()))
      {
        bufferInteg1 += 100 / lidarBuffer.size();  // Scale to buffer size
      }
      if ((lidarBuffer[i] < config.lidarZone2Max.toInt()) &&
          (lidarBuffer[i] > config.lidarZone2Min.toInt()))
      {
        bufferInteg2 += 100 / lidarBuffer.size();
      }
    }

    // Test for car-ness
    if ( bufferInteg1 > threshold) { zone1Strength = 100; } // We have Car!
    if ( bufferInteg2 > threshold) { zone2Strength = 100; }

    // Cars at longer distances than the zoneMax take away zoneStrength
    // Experimenting with an exponential decay. 
    // Hardcoded decay constant. TODO: If this works well, make adjustable.
    if (tfDist > config.lidarZone1Max.toInt())
    {
      zone1Strength = zone1Strength -  zone1Strength * 0.05; // Subtract 'car-ness' from Zone 1
    }

    if (tfDist > config.lidarZone2Max.toInt()) 
    {
      zone2Strength = zone2Strength - zone2Strength *0.05; // Subtract 'car-ness' from Zone 2
    }
    
    previousCarPresentLane1 = carPresentLane1;
    previousCarPresentLane2 = carPresentLane2;

    // Hardcoded cutoff for the exponential decay. Report once car-ness decays to 10%
    carPresentLane1 = (zone1Strength > 10);
    carPresentLane2 = (zone2Strength > 10);

    if ((previousCarPresentLane1 == true) && (carPresentLane1 == false)) 
    { // The car was here and now has left the field of view.
      carEvent1 = 300;
      retValue = 1;
    }
    else
    {
      carEvent1 = 0;
    }

    if ((previousCarPresentLane2 == true) && (carPresentLane2 == false))    
    { // The car was here and now has left the field of view.
      carEvent2 = 300;
      retValue = 2;
    }
    else
    {
      carEvent2 = 0;
    }

// For the serial plotter.
     if (config.showDataStream == "true"){
        debugUART.print(tfDist);
        debugUART.print(",");
        debugUART.print(config.lidarZone1Max.toFloat());
        debugUART.print(",");
        debugUART.print(config.lidarZone1Min.toFloat());
        debugUART.print(",");
        debugUART.print(config.lidarZone2Max.toFloat());
        debugUART.print(",");
        debugUART.print(config.lidarZone2Min.toFloat());
        debugUART.print(",");
        debugUART.print(carEvent1);
        debugUART.print(",");
        debugUART.print(carEvent2);
        debugUART.print(",");
        debugUART.print(zone1Strength);
        debugUART.print(",");
        debugUART.println(zone2Strength);
     }

  }
  else
  {
    // Report the error
    // tfmP.printStatus();
    // Other than TFMP_WEAK, the other error I see occassionaly is 'CHECKSUM'
    // Why would serial com have a problem?...
    // TODO: Investigate.
  }


  tfmP.sendCommand(TRIGGER_DETECTION, 0); // Trigger a LIDAR measurment
  return retValue;

}

#endif // __DIGAME_LIDAR_H__