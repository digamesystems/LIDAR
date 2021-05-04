// Display Library example for SPI e-paper panels from Dalian Good Display and boards from Waveshare.
// Requires HW SPI and Adafruit_GFX. Caution: the e-paper panels require 3.3V supply AND data lines!
//
// Display Library based on Demo Example from Good Display: http://www.e-paper-display.com/download_list/downloadcategoryid=34&isMode=false.html
//
// Author: Jean-Marc Zingg
//
// Version: see library.properties
//
// Library: https://github.com/ZinggJM/GxEPD2

// Supporting Arduino Forum Topics:
// Waveshare e-paper displays with SPI: http://forum.arduino.cc/index.php?topic=487007.0
// Good Display ePaper for Arduino: https://forum.arduino.cc/index.php?topic=436411.0

// see GxEPD2_wiring_examples.h for wiring suggestions and examples

// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0

// uncomment next line to use class GFX of library GFX_Root instead of Adafruit_GFX
//#include <GFX.h>
// Note: if you use this with ENABLE_GxEPD2_GFX 1:
//       uncomment it in GxEPD2_GFX.h too, or add #include <GFX.h> before any #include <GxEPD2_GFX.h>

#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>

// select the display constructor line in one of the following files (old style):
#include "GxEPD2_display_selection.h"
#include "GxEPD2_display_selection_added.h"
//#include "GxEPD2_display_selection_more.h" // private

// or select the display class and display driver class in the following file (new style):
//#include "GxEPD2_display_selection_new_style.h"

#include "bitmaps/Bitmaps200x200.h" // 1.54" b/w
#include "bitmaps/Bitmaps1304x984.h" // 12.48" b/w // For some reason it needs this header to work... ???

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("setup");
  delay(100);
  display.init(115200);
  // first update should be full refresh
  initDisplay();
  
  // partial refresh mode can be used to full screen,
  // effective if display panel hasFastPartialUpdate
  helloFullScreenPartialMode();

  delay(1000);
  double val = 0;
  for (int i =0; i< 6; i++){
     showValue(val, 0);
     val++;
     //delay(1000);
  }
  delay(1000);
  
  Serial.println("setup done");
}

void loop()
{
}

// note for partial update window and setPartialWindow() method:
// partial update window size and position is on byte boundary in physical x direction
// the size is increased in setPartialWindow() if x or w are not multiple of 8 for even rotation, y or h for odd rotation
// see also comment in GxEPD2_BW.h, GxEPD2_3C.h or GxEPD2_GFX.h for method setPartialWindow()

const char strVehicleCount[]   = "Vehicle Count";
//const char HelloArduino[] = "Hello Arduino!";
const char HelloEpaper[]  = "Hello E-Paper!";

void initDisplay()
{
  display.setRotation(1);
  display.setFont(&FreeMonoBold12pt7b);
  display.setTextColor(GxEPD_BLACK);
  display.setFullWindow();
  display.firstPage();
  return;
}

void helloFullScreenPartialMode()
{
  
  //Serial.println("helloFullScreenPartialMode");
  const char title[] = "Digame Systems LIDAR Vehicle Counter v 0.95\n";
 
  display.setPartialWindow(0, 0, display.width(), display.height());
 
  
  // do this outside of the loop
  int16_t tbx, tby; uint16_t tbw, tbh;
  // center update text
  display.getTextBounds(title, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t utx = ((display.width() - tbw) / 2) - tbx;
  uint16_t uty = ((display.height() / 4) - tbh / 2) - tby;
  // center update mode

  // center HelloWorld
  display.getTextBounds(strVehicleCount, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t hwx = ((display.width() - tbw) / 2) - tbx;
  uint16_t hwy = ((display.height() - tbh) / 2) - tby ;
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);

    display.setCursor(utx, uty);    
    display.print(title);
    
    display.setCursor(hwx, hwy + 20);
    display.print(strVehicleCount);
    
  }
  while (display.nextPage());
  //Serial.println("helloFullScreenPartialMode done");
}


#if defined(ESP8266) || defined(ESP32)
#include <StreamString.h>
#define PrintString StreamString
#else
class PrintString : public Print, public String
{
  public:
    size_t write(uint8_t data) override
    {
      return concat(char(data));
    };
};
#endif

void showValue(double v, int digits)
{
  //Serial.println("helloValue");
  display.setRotation(1);
  display.setFont(&FreeMonoBold18pt7b);
  display.setTextColor(display.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
  PrintString valueString;
  valueString.print(v, digits);
  int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds(valueString, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t x = ((display.width() - tbw) / 2) - tbx;
  uint16_t y = (display.height() * 3 / 4) + tbh / 2; // y is base line!
  // show what happens, if we use the bounding box for partial window
  uint16_t wx = (display.width() - tbw) / 2;
  uint16_t wy = (display.height() * 3 / 4) - tbh / 2;
  display.setPartialWindow(wx, wy, tbw, tbh);
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y);
    display.print(valueString);
  }
  while (display.nextPage());
  //delay(2000);
  // make the partial window big enough to cover the previous text
  uint16_t ww = tbw; // remember window width
  display.getTextBounds(HelloEpaper, 0, 0, &tbx, &tby, &tbw, &tbh);
  // adjust, because HelloEpaper was aligned, not centered (could calculate this to be precise)
  ww = max(ww, uint16_t(tbw + 12)); // 12 seems ok
  wx = (display.width() - tbw) / 2;
  // make the window big enough to cover (overwrite) descenders of previous text
  uint16_t wh = FreeMonoBold18pt7b.yAdvance;
  wy = (display.height() * 3 / 4) - wh / 2;
  display.setPartialWindow(wx, wy, ww, wh);
  // alternately use the whole width for partial window
  //display.setPartialWindow(0, wy, display.width(), wh);
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y);
    display.print(valueString);
  }
  while (display.nextPage());
  //Serial.println("helloValue done");
}
