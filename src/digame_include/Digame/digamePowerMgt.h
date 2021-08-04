/*
  Power management functions
*/
#ifndef __DIGAME_POWER_MGT_H__
#define __DIGAME_POWER_MGT_H__

#include <WiFi.h>         // WiFi stack
#include "driver/adc.h"   // ADC functions. (Allows us to turn off to save power.)
#include <esp_bt.h>       // Bluetooth control functions
#include <esp_wifi.h>

#define debugUART Serial


//************************************************************************
void setLowPowerMode() {
    debugUART.print("  Switching to Low Power Mode... ");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    btStop();
    adc_power_off();
    esp_wifi_stop();
    esp_bt_controller_disable();
    setCpuFrequencyMhz(40); // Slow down the CPU
    debugUART.println("Done. Low Power Mode Enabled.");
}


//************************************************************************ 
void setFullPowerMode() {
    debugUART.print("Switching to Full Power Mode... ");
    setCpuFrequencyMhz(240); // Speed up the CPU
    btStart();               // Turn on Bluetooth
    adc_power_on();          // Turn on the ADCs
    WiFi.mode(WIFI_STA);
    debugUART.print("Done. Full Power Mode Enabled. ");
}

#endif 

