#ifndef ENCODER_SETUP_H
#define ENCODER_SETUP_H
#include <Arduino.h>
#include "driver/periph_ctrl.h"


class QuadEncoder {
public:
  int clkPin, dirPin;
  volatile long tickCount;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

  QuadEncoder(int clk_pin, int dir_pin, float ppr);

  void clearTickCounts();
  void updateCounts();
  long getTotalCounts();
  float getCountsPerSec();
  void setPulsePerRev(float ppr);
  float getAngPos();
  float getAngVel();

private:
  float pulsePerRev;
  long prevTickCount;
  long totalCount;
  float countsPerSec;

  uint64_t prev_us;
};


#endif