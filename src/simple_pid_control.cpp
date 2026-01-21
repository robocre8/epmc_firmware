#include "simple_pid_control.h"

SimplePID::SimplePID(float Kp, float Ki, float Kd, float out_min, float out_max)
{
  reset();

  kp = Kp;
  ki = Ki;
  kd = Kd;
  outMax = out_max;
  outMin = out_min;
}

void SimplePID::setParameters(float Kp, float Ki, float Kd, float out_min, float out_max)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;
  outMax = out_max;
  outMin = out_min;
}

void SimplePID::setGains(float Kp, float Ki, float Kd)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

void SimplePID::setKp(float Kp)
{
  kp = Kp;
}

void SimplePID::setKi(float Ki)
{
  ki = Ki;
}

void SimplePID::setKd(float Kd)
{
  kd = Kd;
}

void SimplePID::setOutLimit(float out_max, float out_min)
{
  outMax = out_max;
  outMin = out_min;
}

void SimplePID::begin()
{
  reset();
}

float SimplePID::compute(float target, float input)
{
  float dt = (float)(esp_timer_get_time() - lastTime)/1000000.0;

  p_error = target - input;

  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  d_error = ((target - prevTarget) - (input - prevInput))/dt;

  output = (kp * p_error) + i_term + (kd * d_error);

  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= outMax)
    output = outMax;
  else if (output <= outMin)
    output = outMin;
  else
    /*
    * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    i_term += ki * p_error * dt;

  prevTarget = target;
  prevInput = input;
  lastTime = esp_timer_get_time();

  return output;
}

void SimplePID::reset()
{
  output = 0.0;
  prevInput = 0.0;
  prevTarget = 0.0;
  i_term = 0.0;
  lastTime = esp_timer_get_time();
}