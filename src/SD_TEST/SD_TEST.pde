//
// Title        : FileLogger library for Arduino, example code
// Author       : Eduardo GarcÃ¯Â¿Â½a (egarcia@stream18.com)
// Date         : April 2009
// Id			: $Id: FileLoggerDemo.pde 24 2009-04-23 22:45:13Z stream18 $
//
// DISCLAIMER:
// The author is in no way responsible for any problems or damage caused by
// using this code. Use at your own risk.
//
// LICENSE:
// This code is distributed under the GNU Public License
// which can be found at http://www.gnu.org/licenses/gpl.txt
//


#include "FileLogger.h"

#define LED_GREEN 5
#define LED_YELLOW 6
#define SD_CARD_DETECT 17

#define MESSAGE "Hello, this is my message. Just testing the FileLogger library.\r\n"
unsigned long length = sizeof(MESSAGE)-1;
byte buffer[] = MESSAGE;

void setup(void) {
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT); 
  pinMode(SD_CARD_DETECT, INPUT);
  digitalWrite(SD_CARD_DETECT, HIGH);  //pullups
   digitalWrite(LED_GREEN, LOW);  // turn on
    digitalWrite(LED_YELLOW, HIGH);  // turn off
}

void loop(void) {
  if(digitalRead(SD_CARD_DETECT)==LOW){
    digitalWrite(LED_GREEN, HIGH);  // turn off
    digitalWrite(LED_YELLOW, LOW);  // turn on
    for(int i=0; i<10; i++) {
      FileLogger::append("data.txt", buffer, length);
    }
    digitalWrite(LED_YELLOW, HIGH);  // turn off
    digitalWrite(LED_GREEN, LOW);  // turn on
  }
  delay(5000);
}



