#ifndef __DIGAME_LIDAR_H__
#define __DIGAME_LIDAR_H__

#define debugUART Serial
#define tfMiniUART Serial2

#include <digameJSONConfig.h> // Program parameters from config file on SD card
#include <TFMPlus.h>          // Include TFMini Plus LIDAR Library v1.4.0
TFMPlus tfmP;                 // Create a TFMini Plus object

#include <CircularBuffer.h>   // Adafruit library. Pretty small!

const int lidarSamples = 100;
CircularBuffer<int, lidarSamples> lidarBuffer; // We're going to hang onto the last 100 raw data 
                                               //   points to visualize what the sensor sees

const int histogramSize = 120; // Playing with a histogram of distances to see if we can learn
                               //   how to determine lane posistions on our own. 10 cm bins
unsigned long lidarDistanceHistogram[histogramSize];

unsigned long lidarTimeHistogram[histogramSize]; // 0-12 seconds in tenth of a second bins

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
      debugUART.printf("  Adjusting Frame Rate... ");
      if (tfmP.sendCommand(SET_FRAME_RATE, FRAME_0))
      { //FRAME_0 is triggered mode.
        debugUART.println("  Frame Rate Adjusted.");
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
    retValue = retValue + String(i*10) + ", " + String(lidarDistanceHistogram[i]) + "\n";
  }
  return retValue;
}

//*****************************************************************************
// A dirt-simple processing scheme based on a hard threshold read from the
// SD card. Pretty susceptible to noisy conditions. TODO: Improve.
bool processLIDARSignal(Config config)
{
  // LIDAR signal analysis parameters

  int16_t tfDist = 0; // Distance to object in centimeters
  int16_t tfFlux = 0; // Strength or quality of return signal
  int16_t tfTemp = 0; // Internal temperature of Lidar sensor chip
  static float smoothed = 0.0; // A smoothed version of the raw distance data

  static bool carPresentLane1 = false;         // Do we see a car now?
  static bool previousCarPresentLane1 = false; // Had we seen a car last time?
  
  static bool carPresentLane2 = false;         // Do we see a car now?
  static bool previousCarPresentLane2 = false; // Had we seen a car last time?

  static unsigned long firstInRangeMS1 = 0; // The time in ms, the car first got
  static unsigned long firstInRangeMS2 = 0; // close enough to count as 'present'

  static unsigned long timeInRange1 = 0; // How long has the car close enough to be 'present'
  static unsigned long timeInRange2 = 0;

  unsigned int carEvent = 0;            // A variable for the serial plotter.
  unsigned int lidarUpdateRate = 15;    // Time in ms between readings
  unsigned long minTimeInRange = (unsigned long)config.lidarResidenceTime.toInt();  // Minimum time to count as fully 'present'.

  bool retValue = false; // Return value. Do we have a vehicle event?

  tfmP.sendCommand(TRIGGER_DETECTION, 0); // Trigger a LIDAR measurment
  delay(lidarUpdateRate);                 //

  // Read the LIDAR Sensor
  if (!(tfmP.getData(tfDist, tfFlux, tfTemp))) // Returns false on an error.
  {
    tfDist = 1200; // Our typical error on a failed 'getData' is 'low signal' suggesting no
                    // reflection back.
                    // Experimenting with just returning 12 meters (infinity)
  }

  // When very close, or looking off into empty space, the sensor reports Zero.
  // The short range isn't an issue for us.
  // Any Zeros we see will be due to no reflective target in range.
  // Assume max range for the sensor to get the threshold algorithm to perform properly.
  if (tfDist == 0)
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



  if ( ((previousCarPresentLane1 == true) && (carPresentLane1 == false)) ||
       ((previousCarPresentLane2 == true) && (carPresentLane2 == false))
     )
  { // The car has left the field of view.
    carEvent = 300;
    retValue = true;
  }
  else
  {
    carEvent = 0;
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
  debugUART.println(carEvent);
#endif

  previousCarPresentLane1 = carPresentLane1;
  previousCarPresentLane2 = carPresentLane2;

  return retValue;
}

#endif // __DIGAME_LIDAR_H__