#ifndef Battery_h
#define Battery_h

#include "WProgram.h"

typedef void (*callback_h)(void);

#define BATTERY_STATUS_NORMAL 0
#define BATTERY_STATUS_LOW 1

#define max_voltage  9240   // max voltage range  x 1000  should be value that is at 100% of voltage divider i..e the max voltage that the divider was calculated

class Battery
{
public:
  Battery(int adc_pin, int en_pin, int lowVoltThreshold);
  void update(void);
  int getRawVoltage(void);
  int getVoltage(void);
  int getStatus(void);

  void setLowVoltageThreshold(int threshold);

  void (*onLowVoltage) (void);
  void setOnLowVoltageCallback(callback_h callback);

private:
  int status;
  int raw_voltage;
  int adc_pin;
  int en_pin;
  int lowVoltageThreshold;
};

#endif

