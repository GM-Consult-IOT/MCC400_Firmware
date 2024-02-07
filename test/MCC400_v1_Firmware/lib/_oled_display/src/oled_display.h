#include <Arduino.h>
#include "SSD1306Wire.h"        // For a connection via I2C using the Arduino Wire include:
#include <U8g2lib.h>
#include <inttypes.h>
#include <Wire.h>

// Initialize the OLED display using Arduino Wire:
// SSD1306Wire display(0x3c, SDA, SCL);


char char_arr[128];

void toChar(String str){  
   strcpy(char_arr, str.c_str());
}


U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
void oledInit(){
      // Initialising the UI will init the display too.
  display.begin();
  // flip the screen
  // display.flipScreenVertically();
  // set the default font
  // display.setFont(ArialMT_Plain_10);
}

// Displays a splash screen for the device
void oledSplash(String label, String data){
  display.clearBuffer();
   toChar(label);
  display.setFont(u8g2_font_helvR10_tf);
  display.drawStr(4, 12,char_arr);
   display.setFont(u8g2_font_helvR24_tf);
   toChar(data);
  display.drawStr(4, 48, char_arr);
  // display.display();
    display.sendBuffer();	
  delay(2000);
}


void oledWrite(String label, String data){
  display.clearBuffer();
  // display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(u8g2_font_helvR18_tf);
  toChar(label);
  display.drawStr(4, 24, char_arr);
  display.setFont(u8g2_font_helvR18_tf);
  toChar(data);
  display.drawStr(4, 48, char_arr);
  // display.display();
    display.sendBuffer();	
}

void oledClear(){
  display.clearBuffer();
  display.sendBuffer();
}




