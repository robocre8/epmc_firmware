#ifndef ENCODER_SETUP_H
#define ENCODER_SETUP_H
#include <Arduino.h>
#include "driver/periph_ctrl.h"

// For critical sections on ESP32
static portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;


class QuadEncoder {
public:
  int clkPin, dirPin;
  float pulsePerRev;
  volatile long tickCount;
  float freqPerTick;
  volatile float frequency;
  volatile uint64_t oldFreqTime, checkFreqTime, freqSampleTime=25000;

  QuadEncoder(int clk_pin, int dir_pin, float ppr);

  void clearTickCounts();
  void setPulsePerRev(float ppr);
  float getAngPos();
  float getAngVel();
  void setStopFreqInUs(uint64_t freq);
  void resetFrequency();
};


#endif


