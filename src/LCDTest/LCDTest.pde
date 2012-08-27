#include <LCDdogmSPI.h>

#define CSB_PIN 16
#define RS_PIN 14
#define LCD_LINES 2
#define LCD_BL 15

// This initializes for a 2 LINES display

LCDdogmSPI lcd = LCDdogmSPI(LCD_LINES, CSB_PIN, RS_PIN, LCD_BL);

void setup() { 
  lcd.init();
  lcd.clear();
  lcd.backlightOn();
  lcd.cursorTo(0,0);
  lcd.println("Rallylog");
  lcd.cursorTo(1,8);
  lcd.println("ver 0.1a");
  delay(10000);
  lcd.backlightOff();
}

void loop() {
  delay(10); 
}
