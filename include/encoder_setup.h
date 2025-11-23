#ifndef ENCODER_SETUP_H
#define ENCODER_SETUP_H
#include <Arduino.h>
#include "driver/periph_ctrl.h"

// For critical sections on ESP32
static portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;


class QuadEncoder {
public:
  int clkPin, dirPin;
  double pulsePerRev;
  volatile long tickCount;
  double freqPerTick;
  volatile double frequency;
  volatile uint64_t oldFreqTime, checkFreqTime, freqSampleTime=25000;

  QuadEncoder(int clk_pin, int dir_pin, float ppr);

  void setPulsePerRev(double ppr);
  double getAngPos();
  double getAngVel();
  void setStopFreqInUs(uint64_t freq);
  void resetFrequency();
};


#endif


