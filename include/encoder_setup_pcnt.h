#ifndef ENCODER_SETUP_PCNT_H
#define ENCODER_SETUP_PCNT_H
#include <Arduino.h>
#include "driver/pcnt.h"
#include "esp_timer.h"


class QuadEncoderPCNT {

public:
  QuadEncoderPCNT(int pulse_pin, int ctrl_pin, float ppr, pcnt_unit_t unit, int16_t high_lim = 32767, int16_t low_lim  = -32768);

  void begin();
  void setPulsePerRev(float ppr);
  int32_t getTotalCount();
  void clearCount();
  float getCountsPerSec();
  float getAngPos();
  float getAngVel();
  void update_total_encoder_count();

private:
  int16_t high_lim_, low_lim_;

  int pulsePin, ctrlPin;
  float pulsePerRev, countsPerSec;
  int32_t prevCount, totalCount;
  pcnt_unit_t unit_;

  int64_t prev_us;

  volatile int32_t accum_ = 0;

  static QuadEncoderPCNT* instance_table[PCNT_UNIT_MAX];

  static void IRAM_ATTR pcnt_isr(void* arg);

  int32_t getCount();
};


#endif


