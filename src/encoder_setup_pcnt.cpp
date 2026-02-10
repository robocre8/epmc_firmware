#include "encoder_setup_pcnt.h"

QuadEncoderPCNT* QuadEncoderPCNT::instance_table[PCNT_UNIT_MAX] = {nullptr};

QuadEncoderPCNT::QuadEncoderPCNT(int pulse_pin, int ctrl_pin, float ppr, pcnt_unit_t unit, int16_t high_lim, int16_t low_lim)
{
  pulsePin = pulse_pin;
  ctrlPin = ctrl_pin;
  pulsePerRev = ppr;
  unit_ = unit;

  high_lim_ = high_lim;
  low_lim_ = low_lim;

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
  cfg.unit           = unit_;

  // Count on rising edge only (minimal)
  cfg.pos_mode = PCNT_COUNT_INC;
  cfg.neg_mode = PCNT_COUNT_DIS;

  // Direction control from ctrl pin
  cfg.lctrl_mode = PCNT_MODE_REVERSE;
  cfg.hctrl_mode = PCNT_MODE_KEEP;

  cfg.counter_h_lim = high_lim_;
  cfg.counter_l_lim = low_lim_;

  pcnt_unit_config(&cfg);

  // Optional glitch filter (very recommended)
  pcnt_set_filter_value(unit_, 100); // ~1.25us
  pcnt_filter_enable(unit_);

  pcnt_event_enable(unit_, PCNT_EVT_H_LIM);
  pcnt_event_enable(unit_, PCNT_EVT_L_LIM);

  pcnt_counter_pause(unit_);
  pcnt_counter_clear(unit_);
  pcnt_counter_resume(unit_);

  instance_table[unit_] = this;

  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(unit_, pcnt_isr, (void*)unit_);
}

void IRAM_ATTR QuadEncoderPCNT::pcnt_isr(void* arg)
{
  pcnt_unit_t unit = (pcnt_unit_t)(int)arg;
  QuadEncoderPCNT* inst = instance_table[unit];
  if (!inst) return;

  uint32_t status = 0;
  pcnt_get_event_status(unit, &status);

  if (status & PCNT_EVT_H_LIM) {
      inst->accum_ += inst->high_lim_;
  }

  if (status & PCNT_EVT_L_LIM) {
      inst->accum_ += inst->low_lim_;
  }
}

void QuadEncoderPCNT::setPulsePerRev(float ppr)
{
  pulsePerRev = ppr;
}

int32_t QuadEncoderPCNT::getCount()
{
  int16_t raw;
  pcnt_get_counter_value(unit_, &raw);

  int32_t acc;
  noInterrupts();
  acc = accum_;
  interrupts();

  return acc + raw;
}


void QuadEncoderPCNT::clearCount()
{
  pcnt_counter_clear(unit_);

  noInterrupts();
  accum_ = 0;
  interrupts();

  prevCount = 0;
  totalCount = 0;
  countsPerSec = 0.0;
  prev_us = esp_timer_get_time();
}

void QuadEncoderPCNT::update_total_encoder_count()
{
  int64_t current_us = esp_timer_get_time();
  int32_t currentCount  = getCount();
  int32_t deltaCount = currentCount - prevCount;
  int64_t dt = current_us - prev_us;

  if (dt > 0)
    countsPerSec = (float)deltaCount * (1000000.0 / (float)dt);
  else
    countsPerSec = 0.0f;

  totalCount = currentCount;

  prevCount = currentCount;
  prev_us = current_us;
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