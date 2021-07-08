/*Functions for using the Adafruit 1.54" Monochrome eInk Display. 
 * 
 * See: https://www.adafruit.com/product/4196
 * 
 */
 
#ifndef __DIGAME_DISPLAY_H__
#define __DIGAME_DISPLAY_H__

// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0

#include <GxEPD2_BW.h>

//#include "GxEPD2_display_selection.h"

// Our pinouts to the Adafruit display. 
#define EPD_DC      5   // can be any pin, but required!
#define EPD_CS      26  // can be any pin, but required!
#define EPD_BUSY    2   // can set to -1 to not use a pin (will wait a fixed delay)
#define SRAM_CS     27  // can set to -1 to not use a pin (uses a lot of RAM!)
#define SD_CS       14  // SD card chip select
#define EPD_RESET   0   // can set to -1 and share with chip Reset (can't deep sleep)


// Just grabbed the constructor we need from GxEPD2_display_selection and copied here.
GxEPD2_BW<GxEPD2_154, GxEPD2_154::HEIGHT> display(GxEPD2_154(/*CS=5*/ EPD_CS, /*DC=*/ EPD_DC, /*RST=*/ EPD_RESET, /*BUSY=*/ EPD_BUSY)); // GDEH0154D67

#define debugUART Serial

void initDisplay();
void displaySplash();
void displayInitializing();
void displayCount(double c);
void showValue(double v);


//******************************************************************************************
void initDisplay()
{
  display.init(0);
  display.setRotation(3);
  display.setTextSize(2);
  //display.setFont(&FreeMonoBold9pt7b);
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
void centerPrint(String s, uint16_t y){
  int16_t tx, ty; 
  uint16_t tw, th;
  display.getTextBounds(s, 0, 0, &tx, &ty, &tw, &th);
  uint16_t utx = ((display.width() - tw) / 2) - tx;
  uint16_t uty = y;   //((display.height() - th) / 2) - ty;
  display.setCursor(utx, uty);
  display.print(s);
}

//******************************************************************************************
void displayTitles(String title1, String title2){
  display.fillScreen(GxEPD_WHITE);
  //display.display();   
  //display.setTextColor(GxEPD_BLACK);      
  display.setTextSize(3);
  centerPrint(title1,10);
  display.setTextSize(2);
  centerPrint(title2, 40);    
}

//******************************************************************************************
void displayCopyright(){
  display.setTextSize(1);
  centerPrint("HEIMDALL VCS", 170);
  centerPrint("Copyright 2021, Digame Systems.", 180);
  centerPrint("All rights reserved.", 190);
  display.display();    
}

//******************************************************************************************
void displaySplashScreen(String swVersion){
  displayTitles("HEIMDALL","(LIDAR Sensor)");
  centerPrint("Vehicle", 70);
  centerPrint("Counting System", 90);
  centerPrint("Version", 110);
  centerPrint(swVersion, 130);
  displayCopyright(); 
}

//******************************************************************************************
void displayIPScreen(String s){
  displayTitles("NETWORK","");
  centerPrint("IP Address",75);
  centerPrint(s, 100);
  displayCopyright(); 
}

//******************************************************************************************
void displayAPScreen(String s){
  displayTitles("NETWORK","(ACCESS POINT)"); 
  centerPrint("SSID",70);
  centerPrint("Digame-AP", 90); 
  centerPrint("IP Address",115);
  centerPrint(s, 135);
  displayCopyright(); 
}

//******************************************************************************************
void displayStatusScreen(String s){
  displayTitles("SELF-TEST",""); 
  display.setCursor(0, 50);
  display.print(s);
  displayCopyright();  
}



//******************************************************************************************
void displayCountScreen(double v)
{
  displayTitles("COUNTS","");
  displayCopyright();     
}


//******************************************************************************************
void showValue(double v)
{
  int digits = 0; 
  
  //display.setRotation(0);
  //display.setFont(&FreeMonoBold18pt7b);
  display.setTextSize(5);
  display.setTextColor(GxEPD_BLACK);
  
  PrintString valueString;
  valueString.print(v, digits);
  
  int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds(valueString, 0, 0, &tbx, &tby, &tbw, &tbh);
  /*
  debugUART.print("tbx: ");
  debugUART.print(tbx);  
  debugUART.print(" tby: ");
  debugUART.print(tby);  
  debugUART.print(" tbw: ");
  debugUART.print(tbw);  
  debugUART.print(" tbh: ");
  debugUART.println(tbh);  
  */
  uint16_t x = ((display.width() - tbw) / 2) - tbx;
  uint16_t y = ((display.height() - tbh) / 2) - tby;  //+ tbh / 2; // y is base line!
  /*
  debugUART.print("x: ");
  debugUART.print(x);
  debugUART.print(" y: ");
  debugUART.println(y);
  */
  
  // show what happens, if we use the bounding box for partial window
  uint16_t wx = (display.width() - tbw) / 2;
  uint16_t wy = (display.height()- tbh) / 2;  // / 2;
  
  display.setPartialWindow(wx, wy, tbw, tbh);
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y);
    display.print(valueString);
  }
  while (display.nextPage());
  
}

#endif //__DIGAME_DISPLAY_H__
