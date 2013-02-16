#include "WProgram.h"
#include "Battery.h"

Battery::Battery(int adc_pin, int en_pin, int lowVoltThreshold)
{
  this->adc_pin = adc_pin;
  this->en_pin = en_pin;
  pinMode(this->en_pin, OUTPUT);
  digitalWrite(this->en_pin, LOW); 				// turn off
  this->lowVoltageThreshold = lowVoltThreshold;
  this->raw_voltage = 0;
  this->status = BATTERY_STATUS_NORMAL;
}

void Battery::update(void)
{
  digitalWrite(this->en_pin, HIGH);				// turn on monitor circuit
  delay(25);									// stabilise before read
  this->raw_voltage = analogRead(this->adc_pin);	        // read voltage
  digitalWrite(this->en_pin, LOW);				// turn off monitor circuit

  if(getVoltage() <= this->lowVoltageThreshold)	                // check for low voltage condition
  {
    this->status=BATTERY_STATUS_LOW;	

    this->onLowVoltage();					// callback function if low
  } 
  else
  {
    this->status=BATTERY_STATUS_NORMAL;			        // else normal
  }
}

int Battery::getRawVoltage(void)
{
  return this->raw_voltage;
}

int Battery::getVoltage()
{
  return ((this->raw_voltage * 9)/10);			        // scale reading by 100
}

void Battery::setOnLowVoltageCallback(callback_h callback)
{
  this->onLowVoltage = callback;
}

void Battery::setLowVoltageThreshold(int threshold)
{
  this->lowVoltageThreshold = threshold;
}

int Battery::getStatus(void)
{
  return status;
}

