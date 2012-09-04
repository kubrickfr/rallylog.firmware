#include "WProgram.h"
#include "Battery.h"

Battery::Battery(int adc_pin, int en_pin, int lowVoltThreshold)
{
  adc_pin = adc_pin;
  en_pin = en_pin;
  pinMode(en_pin, OUTPUT);
  digitalWrite(en_pin, LOW); 				// turn off
  lowVoltageThreshold = lowVoltThreshold;
  raw_voltage = 0;
  status = BATTERY_STATUS_NORMAL;
}

Battery::Battery(int adc_pin, int en_pin, int lowVoltThreshold,void(*callback)(eBatteryStatus))
{
  Battery(adc_pin,en_pin,lowVoltThreshold);
  onStatusChange = callback;
}

void Battery::update(void)
{
  digitalWrite(en_pin, HIGH);				// turn on monitor circuit
  delay(25);									// stabilise before read
  raw_voltage = analogRead(adc_pin);	        // read voltage
  digitalWrite(en_pin, LOW);				// turn off monitor circuit

  if(getVoltage() <= lowVoltageThreshold)	                // check for low voltage condition
  {
    status=BATTERY_STATUS_LOW;	
    onStatusChange(status);
    onLowVoltage();					// callback function if low
  } 
  else
  {
    status=BATTERY_STATUS_NORMAL;			        // else normal
    onStatusChange(status);
  }
}

int Battery::getRawVoltage(void)
{
  return raw_voltage;
}

int Battery::getVoltage()
{
  return ((raw_voltage * 9)/10);			        // scale reading by 100
}

void Battery::setOnLowVoltageCallback(void(*callback)(void))
{
  onLowVoltage = callback;
}

void Battery::setOnStatusChange(void(*callback)(eBatteryStatus))
{
 onStatusChange = callback;
}

void Battery::setLowVoltageThreshold(int threshold)
{
  lowVoltageThreshold = threshold;
}

eBatteryStatus Battery::getStatus(void)
{
  return status;
}

