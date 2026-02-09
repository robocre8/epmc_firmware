#ifndef ENCODER_SETUP_PCNT_H
#define ENCODER_SETUP_PCNT_H
#include <Arduino.h>
#include "driver/pcnt.h"


class QuadEncoderPCNT {

public:
  QuadEncoderPCNT(int pulse_pin, int ctrl_pin, float ppr, pcnt_unit_t pcnt_unit);

  void begin();
  void setPulsePerRev(float ppr);
  int32_t getTotalCount();
  void clearCount();
  float getCountsPerSec();
  float getAngPos();
  float getAngVel();
  void update_total_encoder_count();

private:
  static const int32_t MAX_INT_RANGE = 32767;
  static const int32_t MIN_INT_RANGE = -32768;
  static const int32_t MAX_DELTA_COUNT_RANGE = 25000; // check PCNT Overflow

  int pulsePin, ctrlPin;
  float pulsePerRev, countsPerSec;
  int32_t prevCount, totalCount;
  pcnt_unit_t unit;

  int64_t prev_us;
};


#endif


