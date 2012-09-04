#ifndef Battery_h
#define Battery_h

#include "WProgram.h"


enum eBatteryStatus{
  BATTERY_STATUS_NORMAL,
  BATTERY_STATUS_LOW
};

#define max_voltage  9240   // max voltage range  x 1000  should be value that is at 100% of voltage divider i..e the max voltage that the divider was calculated

class Battery
{
public:
  Battery(int adc_pin, int en_pin, int lowVoltThreshold);
  Battery(int adc_pin, int en_pin, int lowVoltThreshold,void(*)(eBatteryStatus));
  void update(void);
  int getRawVoltage(void);
  int getVoltage(void);
  eBatteryStatus getStatus(void);

  void setLowVoltageThreshold(int threshold);
  
  void setOnLowVoltageCallback(void(*)(void));
  void setOnStatusChange(void(*)(eBatteryStatus));

private:
  void (*onLowVoltage) (void);
  void (*onStatusChange)(eBatteryStatus);
  
  eBatteryStatus status;
  int raw_voltage;
  int adc_pin;
  int en_pin;
  int lowVoltageThreshold;
};

#endif

