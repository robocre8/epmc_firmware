#include "encoder_setup.h"

QuadEncoder::QuadEncoder(int clk_pin, int dir_pin, float ppr)
{
  clkPin = clk_pin;
  dirPin = dir_pin;
  pulsePerRev = ppr;

  pinMode(clkPin, INPUT_PULLUP);
  pinMode(dirPin, INPUT_PULLUP);

  oldFreqTime = esp_timer_get_time();
  checkFreqTime = esp_timer_get_time();
}

void QuadEncoder::setPulsePerRev(float ppr)
{
  pulsePerRev = ppr;
}

void QuadEncoder::clearTickCounts()
{
  portENTER_CRITICAL(&encoderMux);
  tickCount=0;
  portEXIT_CRITICAL(&encoderMux);
}

float QuadEncoder::getAngPos()
{
  portENTER_CRITICAL(&encoderMux);
  long ticks = tickCount;
  portEXIT_CRITICAL(&encoderMux);
  return (2.00 * PI * (float)ticks) / pulsePerRev;
}

float QuadEncoder::getAngVel()
{
  portENTER_CRITICAL(&encoderMux);
  float freq = frequency;
  portEXIT_CRITICAL(&encoderMux);
  return 2.00 * PI * freq;
}

void QuadEncoder::setStopFreqInUs(uint64_t freq)
{
  freqSampleTime = freq;
}

void QuadEncoder::resetFrequency()
{
  uint64_t now = esp_timer_get_time();
  if (now - checkFreqTime >= freqSampleTime)
  {
    frequency = 0;
    checkFreqTime = now; // update timestamp
  }
}