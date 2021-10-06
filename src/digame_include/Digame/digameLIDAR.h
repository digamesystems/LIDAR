#ifndef __DIGAME_LIDAR_H__
#define __DIGAME_LIDAR_H__

#define debugUART Serial
#define tfMiniUART Serial2
#define LOG_RAW_DATA_TO_SD false // Log the Raw data to the SD Card for analysis.


#include <digameJSONConfig.h> // Program parameters from config file on SD card
#include <TFMPlus.h>          // Include TFMini Plus LIDAR Library v1.4.0
TFMPlus tfmP;                 // Create a TFMini Plus object

#include <CircularBuffer.h> // Adafruit library. Pretty small!

const int lidarSamples = 50;
CircularBuffer<int, lidarSamples> lidarBuffer; // We're going to hang onto the last 100 raw data
                                               //   points to visualize what the sensor sees

CircularBuffer<int, 175> lidarHistoryBuffer; // A longer buffer for visualization of the history
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

  if (tfmP.sendCommand(SYSTEM_RESET, 0))
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
// Save histogram data to SD card.
String getDistanceHistogramString()
{
  String retValue = "D (cm), Counts\n";
  for (int i = 0; i < histogramSize; i++)
  {
    retValue = retValue + String(i * 10) + ", " + String(lidarDistanceHistogram[i]) + "\n";
  }
  return retValue;
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

#if SHOW_DATA_STREAM
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
#endif

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


  if ((lidarResult) || (tfmP.status == TFMP_WEAK)) // Process good measurements and treat weak ones as 'infinity'
  {
    // When very close, or looking off into empty space, the sensor reports zero or a negative value.
    // The short range isn't an issue for us.
    // Any Zeros we see will be due to no reflective target in range.
    if ((tfDist <= 0) || (tfDist >= 1000)) // Added the second check in case we pick up a long-range target. 
    {
      tfDist = 999;
    }

    lidarDistanceHistogram[(unsigned int)(tfDist / 10)]++; // Grabbing a histogram of distances
                                                           // to explore automatic lane determination...
                                                           // The check for >= 1000 above is to avoid
                                                           // running off randomly iinto memory.

    lidarBuffer.push(tfDist); // The circular buffer of LIDAR data for analysis
    lidarHistoryBuffer.push((tfDist)); // A longer history for display.


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

    long zone1Strength = 0;  // A measure of how 'present' a car is in each lane over an interval of time
    long zone2Strength = 0;

    for (int i = 0; i < lidarBuffer.size(); i++)
    {
      if ((lidarBuffer[i] < config.lidarZone1Max.toInt()) &&
          (lidarBuffer[i] > config.lidarZone1Min.toInt()))
      {
        zone1Strength = zone1Strength + 2; 
      }

      if ((lidarBuffer[i] < config.lidarZone2Max.toInt()) &&
          (lidarBuffer[i] > config.lidarZone2Min.toInt()))
      {
        zone2Strength = zone2Strength + 2;       
      }
        
    }

    // Normalize to 100%
    zone1Strength = (100* zone1Strength / lidarBuffer.size());
    zone2Strength = (100* zone2Strength / lidarBuffer.size());

    //if (zone1Strength > 100) zone1Strength= 100;
    //if (zone2Strength > 100) zone2Strength= 100;
    
    int threshold = 5; 

    previousCarPresentLane1 = carPresentLane1;
    previousCarPresentLane2 = carPresentLane2;

    carPresentLane1 = (zone1Strength > threshold);
    carPresentLane2 = (zone2Strength > threshold);

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


    #if SHOW_DATA_STREAM
        debugUART.print(tfDist);
        debugUART.print(",");
        //debugUART.print(smoothed);
        //debugUART.print(",");
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
    #endif

  }
  else
  {

    // Report the error
    // tfmP.printStatus();
  }

  return retValue;

}

#endif // __DIGAME_LIDAR_H__