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

void QuadEncoder::setPulsePerRev(double ppr)
{
  pulsePerRev = ppr;
}

double QuadEncoder::getAngPos()
{
  portENTER_CRITICAL(&encoderMux);
  long ticks = tickCount;
  portEXIT_CRITICAL(&encoderMux);
  return (2.00 * PI * (double)ticks) / pulsePerRev;
}

double QuadEncoder::getAngVel()
{
  portENTER_CRITICAL(&encoderMux);
  double freq = frequency;
  portEXIT_CRITICAL(&encoderMux);
  return 2.00 * PI * freq;
}

void QuadEncoder::setStopFreqInUs(uint64_t freq)
{
  freqSampleTime = freq;
}

void QuadEncoder::resetFrequency()
{
  if (esp_timer_get_time() - checkFreqTime >= freqSampleTime)
  {
    frequency = 0;
  }
}