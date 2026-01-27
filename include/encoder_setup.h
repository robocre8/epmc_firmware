#ifndef ENCODER_SETUP_H
#define ENCODER_SETUP_H
#include <Arduino.h>
#include "driver/periph_ctrl.h"


class QuadEncoder {
public:
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  int clkPin, dirPin;
  float pulsePerRev;
  volatile long tickCount;
  volatile uint64_t periodCount, prevPeriodCount;
  volatile int dir;
  uint64_t checkPeriodCount, periodSampleTime=25000;

  QuadEncoder(int clk_pin, int dir_pin, float ppr);

  void clearTickCounts();
  void setPulsePerRev(float ppr);
  float getAngPos();
  float getAngVel();
  void setStopFreqInUs(uint64_t freq);
  void resetFrequency();
};


#endif