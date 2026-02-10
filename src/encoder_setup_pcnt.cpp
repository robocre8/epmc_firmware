#include "encoder_setup_pcnt.h"

QuadEncoderPCNT::QuadEncoderPCNT(int pulse_pin, int ctrl_pin, float ppr, pcnt_unit_t pcnt_unit)
{
  pulsePin = pulse_pin;
  ctrlPin = ctrl_pin;
  pulsePerRev = ppr;
  unit = pcnt_unit;

  pinMode(pulsePin, INPUT_PULLUP);
  pinMode(ctrlPin, INPUT_PULLUP);

  prevCount = 0;
  totalCount = 0;
  countsPerSec = 0.0;
  prev_us = esp_timer_get_time();
}

void QuadEncoderPCNT::begin()
{
  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = pulsePin;
  cfg.ctrl_gpio_num  = ctrlPin;
  cfg.channel        = PCNT_CHANNEL_0;
  cfg.unit           = unit;

  // Count on rising edge only (minimal)
  cfg.pos_mode = PCNT_COUNT_INC;
  cfg.neg_mode = PCNT_COUNT_DIS;

  // Direction control from ctrl pin
  cfg.lctrl_mode = PCNT_MODE_REVERSE;
  cfg.hctrl_mode = PCNT_MODE_KEEP;

  cfg.counter_h_lim = MAX_INT_RANGE;
  cfg.counter_l_lim = MIN_INT_RANGE;

  pcnt_unit_config(&cfg);

  // Optional glitch filter (very recommended)
  pcnt_set_filter_value(unit, 100); // ~1.25us
  pcnt_filter_enable(unit);

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

void QuadEncoderPCNT::setPulsePerRev(float ppr)
{
  pulsePerRev = ppr;
}

void QuadEncoderPCNT::update_total_encoder_count()
{
  int64_t current_us = esp_timer_get_time();

  int16_t currentCount16bit;
  pcnt_get_counter_value(unit, &currentCount16bit);

  int32_t currentCount  = (int32_t)currentCount16bit;

  int32_t deltaCount = currentCount - prevCount;

  if (abs(deltaCount) > MAX_DELTA_COUNT_RANGE) {
    if (deltaCount < 0){
      deltaCount =  MAX_INT_RANGE + deltaCount;
    }
    else if (deltaCount > 0){
      deltaCount =  MIN_INT_RANGE + deltaCount;
    }
  }

  countsPerSec = (float)deltaCount * (1000000.0 / (float)(current_us-prev_us));

  totalCount += deltaCount;
  prevCount = currentCount;
  prev_us = current_us;
}

void QuadEncoderPCNT::clearCount()
{
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
  
  prevCount = 0;
  totalCount = 0;
  countsPerSec = 0.0;
  prev_us = esp_timer_get_time();
}

int32_t QuadEncoderPCNT::getTotalCount()
{
  return totalCount;
}

float QuadEncoderPCNT::getCountsPerSec()
{
  return countsPerSec;
}

float QuadEncoderPCNT::getAngPos()
{
  return (2.00 * PI * (float)totalCount) / pulsePerRev;
}

float QuadEncoderPCNT::getAngVel()
{
  return (2.00 * PI * countsPerSec) / pulsePerRev;
}