/*Functions for using the Adafruit 1.54" Monochrome eInk Display. 
 * 
 * See: https://www.adafruit.com/product/4196
 * 
 */

#ifndef __DIGAME_DISPLAY_H__
#define __DIGAME_DISPLAY_H__

// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 1 // Took me a longer than it should have to find this! --

#include <digameDebug.h>
#include <GxEPD2_BW.h>
#include <EEPROM.h>

//#include "GxEPD2_display_selection.h"

// Our pinouts to the Adafruit display.
#define EPD_DC 5     // can be any pin, but required!
#define EPD_CS 26    // can be any pin, but required!
#define EPD_BUSY 2   // can set to -1 to not use a pin (will wait a fixed delay)
#define SRAM_CS 27   // can set to -1 to not use a pin (uses a lot of RAM!)
#define SD_CS 14     // SD card chip select
#define EPD_RESET -1 // can set to -1 and share with chip Reset (can't deep sleep)

#define debugUART Serial

// Just grabbed the constructors we need from GxEPD2_display_selection and copied here.

// ADAFRUIT 1.54" ePaper Display. SSD1608
GxEPD2_BW<GxEPD2_154, GxEPD2_154::HEIGHT> display1(GxEPD2_154(/*CS=5*/ EPD_CS, /*DC=*/EPD_DC, /*RST=*/EPD_RESET, /*BUSY=*/EPD_BUSY)); // GDEH0154D67

// ADAFRUIT 1.54" ePaper Display. SSD1681
GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display2(GxEPD2_154_D67(/*CS=5*/ EPD_CS, /*DC=*/EPD_DC, /*RST=*/EPD_RESET, /*BUSY=*/EPD_BUSY)); // GDEW0154M09 200x200

//GxEPD2_GFX& display = display2;
String displayType = "SSD1681";

//******************************************************************************************
GxEPD2_GFX &getDisplay()
{ // So we can use different displays.
  if (displayType == "SSD1608")
  {
    return display1;
  }
  else
  {
    return display2;
  }
}

void initDisplay()
{
  DEBUG_PRINTLN(" Initializing eInk Display...");
  DEBUG_PRINTLN("  Reading EEPROM");

  EEPROM.begin(10);
  delay(1000);

  if (EEPROM.read(0)==255){
    DEBUG_PRINTLN("   ****Display Uninitialized****");
    DEBUG_PRINTLN("   Enter Display Type 1=SSD1608, 2=SSD1681: ");
    
    while (!(debugUART.available())){
      delay(10); // wait for data from the user... 
    }

    String inString = debugUART.readStringUntil('\n');
    inString.trim();
    DEBUG_PRINT("   You entered: ");
    DEBUG_PRINTLN(inString);

    if (inString =="1"){
       displayType = "SSD1608";
       EEPROM.write(0,1);
       EEPROM.commit();
    }

    if (inString == "2"){
       displayType = "SSD1681";
       EEPROM.write(0,2);
       EEPROM.commit();
    }
  } 

  DEBUG_PRINT("   Display Type: ");
  DEBUG_PRINT(EEPROM.read(0));


  if (EEPROM.read(0)==1){displayType="SSD1608";}
  if (EEPROM.read(0)==2){displayType="SSD1681";}

  DEBUG_PRINT(" = ");
  DEBUG_PRINTLN(displayType);

  GxEPD2_GFX &display = getDisplay();

  display.init(0);
  display.setRotation(3);
  display.setTextSize(2);
  display.setTextColor(GxEPD_BLACK);
  display.fillScreen(GxEPD_WHITE);
  return;
}

//******************************************************************************************
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

//******************************************************************************************
// Print a string on the display, centered, at a particular y value.
void centerPrint(String s, uint16_t y)
{
  GxEPD2_GFX &display = getDisplay();
  int16_t tx, ty;
  uint16_t tw, th;
  display.getTextBounds(s, 0, 0, &tx, &ty, &tw, &th);
  uint16_t utx = ((display.width() - tw) / 2) - tx;
  uint16_t uty = y; //((display.height() - th) / 2) - ty;
  display.setCursor(utx, uty);
  display.print(s);
}

//******************************************************************************************
void displayTitles(String title1, String title2)
{
  GxEPD2_GFX &display = getDisplay();
  display.fillScreen(GxEPD_WHITE);
  display.setTextSize(3);
  centerPrint(title1, 10);
  display.setTextSize(2);
  centerPrint(title2, 40);
}

//******************************************************************************************
void displayCopyright()
{
  GxEPD2_GFX &display = getDisplay();
  display.setTextSize(1);
  centerPrint("HEIMDALL VCS Family", 170);
  centerPrint("Copyright 2021, Digame Systems.", 180);
  centerPrint("All rights reserved.", 190);
  display.display();
}

//******************************************************************************************
void displaySplashScreen(String s, String swVersion)
{
  displayTitles("HEIMDALL", s);
  centerPrint("Vehicle", 70);
  centerPrint("Counting System", 90);
  centerPrint("Version", 110);
  centerPrint(swVersion, 130);
  displayCopyright();
}

//******************************************************************************************
void displayIPScreen(String s)
{
  displayTitles("NETWORK", "");
  centerPrint("IP Address", 75);
  centerPrint(s, 100);
  displayCopyright();
}

//******************************************************************************************
void displayAPScreen(String ssid, String ip)
{
  displayTitles("NETWORK", "(ACCESS POINT)");
  centerPrint("SSID", 70);
  centerPrint(ssid, 90);
  centerPrint("IP Address", 115);
  centerPrint(ip, 135);
  displayCopyright();
}

//******************************************************************************************
void displayTextScreen(String title, String s)
{
  GxEPD2_GFX &display = getDisplay();
  display.fillScreen(GxEPD_WHITE);
  display.setTextSize(2);
  centerPrint(title, 15);
  display.setCursor(0, 50);
  display.print(s);
  displayCopyright();
}

//******************************************************************************************
void displayTextScreenLarge(String title, String s)
{
  GxEPD2_GFX &display = getDisplay();
  display.fillScreen(GxEPD_WHITE);
  display.setTextSize(2);
  centerPrint(title, 15);
  display.setTextSize(4);
  display.setCursor(0, 25);
  display.print(s);
  displayCopyright();
}

//******************************************************************************************
void displayStatusScreen(String s)
{
  displayTextScreen("SELF-TEST", s);
}

//******************************************************************************************
void displayEventScreen(String s)
{
  displayTextScreen("EVENT", s);
}

//******************************************************************************************
void displayCountScreen(double v)
{
  displayTitles("COUNTS", "");
  displayCopyright();
}

//******************************************************************************************
void displayCountersSummaryScreen(String total, String summary)
{
  GxEPD2_GFX &display = getDisplay();  
  displayTitles("SUMMARY", "Total: " + total);
  display.setCursor(0, 70);
  display.print(summary);
  displayCopyright();
}

//******************************************************************************************
void displayBarcodeScreen()
{
  initDisplay();
  GxEPD2_GFX &display = getDisplay();  
}


//******************************************************************************************
void showValue(double v)
{
  GxEPD2_GFX &display = getDisplay();
  int digits = 0;

  display.setTextSize(5);
  display.setTextColor(GxEPD_BLACK);

  PrintString valueString;
  valueString.print(v, digits);

  int16_t tbx, tby;
  uint16_t tbw, tbh;
  display.getTextBounds(valueString, 0, 0, &tbx, &tby, &tbw, &tbh);

  uint16_t x = ((display.width() - tbw) / 2) - tbx;
  uint16_t y = ((display.height() - tbh) / 2) - tby; //+ tbh / 2; // y is base line!

  // show what happens, if we use the bounding box for partial window
  uint16_t wx = (display.width() - tbw) / 2;
  uint16_t wy = (display.height() - tbh) / 2; // / 2;

  display.setPartialWindow(wx, wy, tbw, tbh);
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y);
    display.print(valueString);
  } while (display.nextPage());
}

#endif //__DIGAME_DISPLAY_H__
