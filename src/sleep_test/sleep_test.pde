#include <Wire.h>
#include <avr/sleep.h>
//#include <avr/wdt.h>
#include <avr/power.h>
//#include <avr/io.h>


void setup(){

  cli();                   // disable interrupts
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

  sleep_enable();          // enables the sleep bit in the mcucr register
  // turn OFF analog parts
  //ACSR = ACSR & 0b11110111 ; // clearing ACIE prevent analog interupts happening during next command
  //ACSR = ACSR | 0b10000000 ; // set ACD bit powers off analog comparator
  //ADCSRA = ADCSRA & 0b01111111 ;  // clearing ADEN turns off analog digital converter
  //ADMUX &= B00111111;  // Comparator uses AREF/GND and not internally generated references
  //power_adc_disable();
  //power_spi_disable();
  //power_twi_disable();
  //power_usart0_disable();
 
  //sleep_bod_disable();
  
  ADCSRA &= ~bit(ADEN);
      // Turn off BOD in sleep (picopower devices only)
  MCUCR = MCUCR | bit(BODSE) | bit(BODS);
  MCUCR = MCUCR &~ bit(BODSE) | bit(BODS);
  SMCR = bit(SM1) | bit(SE); 
  
  sleep_mode();            // here the device is actually put to sleep!!
}

void loop(){}
