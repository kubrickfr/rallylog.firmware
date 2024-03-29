/*
  DogLcd Library - Hello World
 
  Demonstrates the basic use of a DOGM text LCD display.
 This sketch prints "Hello World!" to the LCD
 and shows the time.
 
 See the online-documention for wiring instuctions
 We assume  the following pins are connected:
 * LCD SI pin to digital pin 2
 * LCD CLK pin to digital pin 3
 * LCD RS pin to digital pin 4
 * LCD CSB pin to digital pin 5
 * LCD RESET pin is not used
 * LCD Backlight pin is not used
 
 http://www.arduino.cc/en/Tutorial/LiquidCrystal
 */


// include the library code:
#include <DogLcd.h>


#define SI_PIN 11
#define SCK_PIN 13
#define CSB_PIN 16
#define RS_PIN 14
#define LCD_BL 15

// initialize the library with the numbers of the interface pins
DogLcd lcd(SI_PIN, SCK_PIN, RS_PIN, CSB_PIN );

void setup() {
  // set up the LCD type and the contrast setting for the display 
  lcd.begin(DOG_LCD_M162, 0x28, DOG_LCD_VCC_3V3);
  // Print a message to the LCD.
  lcd.print("hello, world!");
}

void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis()/1000);
}


