#include "encoder_setup.h"

QuadEncoder::QuadEncoder(int clk_pin, int dir_pin, float ppr)
{
  clkPin = clk_pin;
  dirPin = dir_pin;
  pulsePerRev = ppr;

  pinMode(clkPin, INPUT_PULLUP);
  pinMode(dirPin, INPUT_PULLUP);

  portENTER_CRITICAL(&mux);
  tickCount=0;
  portEXIT_CRITICAL(&mux);
  prevTickCount = 0;
  totalCount = 0;
  countsPerSec = 0.0;
  prev_us = esp_timer_get_time();
}

void QuadEncoder::setPulsePerRev(float ppr)
{
  pulsePerRev = ppr;
}

void QuadEncoder::clearTickCounts()
{
  portENTER_CRITICAL(&mux);
  tickCount=0;
  portEXIT_CRITICAL(&mux);
  prevTickCount = 0;
  totalCount = 0;
  countsPerSec = 0.0;
  prev_us = esp_timer_get_time();
}

void QuadEncoder::updateCounts()
{
  portENTER_CRITICAL(&mux);
  long currentCount = tickCount;
  portEXIT_CRITICAL(&mux);

  int64_t current_us = esp_timer_get_time();

  long deltaCount = currentCount - prevTickCount;

  int64_t dt = current_us - prev_us;
  if (dt > 0)
    countsPerSec = (float)deltaCount * (1000000.0 / (float)dt);
  else
    countsPerSec = 0.0f;

  totalCount += deltaCount;
  prevTickCount = currentCount;
  prev_us = current_us;
}

long QuadEncoder::getTotalCounts()
{
  return totalCount;
}

float QuadEncoder::getCountsPerSec()
{
  return countsPerSec;
}

float QuadEncoder::getAngVel()
{
  return (2.00 * PI * countsPerSec) / pulsePerRev;
}

float QuadEncoder::getAngPos()
{
  return (2.00 * PI * (float)totalCount) / pulsePerRev;
}