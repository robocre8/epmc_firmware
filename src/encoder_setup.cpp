#include "encoder_setup.h"

QuadEncoder::QuadEncoder(int clk_pin, int dir_pin, float ppr)
{
  clkPin = clk_pin;
  dirPin = dir_pin;
  pulsePerRev = ppr;

  pinMode(clkPin, INPUT_PULLUP);
  pinMode(dirPin, INPUT_PULLUP);

  tickCount = 0;
  dir = 1;
  periodCount = 0;
  prevPeriodCount = 0;
  checkPeriodCount = esp_timer_get_time();
}

void QuadEncoder::setPulsePerRev(float ppr)
{
  pulsePerRev = ppr;
}

void QuadEncoder::clearTickCounts()
{
  portENTER_CRITICAL(&mux);
  tickCount=0;
  periodCount=0;
  prevPeriodCount=0;
  portEXIT_CRITICAL(&mux);
}

float QuadEncoder::getAngPos()
{
  portENTER_CRITICAL(&mux);
  long ticks = tickCount;
  portEXIT_CRITICAL(&mux);
  return (2.00 * PI * (float)ticks) / pulsePerRev;
}

float QuadEncoder::getAngVel()
{
  uint64_t period_, prevPeriod_;
  int dir_;

  portENTER_CRITICAL(&mux);
  period_     = periodCount;
  prevPeriod_ = prevPeriodCount;
  dir_        = dir;
  portEXIT_CRITICAL(&mux);

  // Guard against resetFrequency() or startup
  if (period_ == 0 || prevPeriod_ == 0) {
    return 0.0f;
  }

  uint64_t dt = period_ - prevPeriod_;
  if (dt == 0) {
    return 0.0f;
  }

  float revPerSec = (1e6f / (float)dt) / pulsePerRev;
  return (dir_ < 0 ? -1.0f : 1.0f) * 2.0f * PI * revPerSec;
}

void QuadEncoder::setStopFreqInUs(uint64_t val)
{
  periodSampleTime = val;
}

void QuadEncoder::resetFrequency()
{
  uint64_t now = esp_timer_get_time();
  if (now - checkPeriodCount >= periodSampleTime)
  {
    portENTER_CRITICAL(&mux);
    periodCount = 0;
    prevPeriodCount = 0;
    checkPeriodCount = now; // update timestamp
    portEXIT_CRITICAL(&mux);
  }
}