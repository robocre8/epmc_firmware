#include "simple_pid_control.h"

SimplePID::SimplePID(double Kp, double Ki, double Kd, double out_min, double out_max)
{
  reset();

  kp = Kp;
  ki = Ki;
  kd = Kd;
  outMax = out_max;
  outMin = out_min;
}

void SimplePID::setParameters(double Kp, double Ki, double Kd, double out_min, double out_max)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;
  outMax = out_max;
  outMin = out_min;
}

void SimplePID::setGains(double Kp, double Ki, double Kd)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

void SimplePID::setKp(double Kp)
{
  kp = Kp;
}

void SimplePID::setKi(double Ki)
{
  ki = Ki;
}

void SimplePID::setKd(double Kd)
{
  kd = Kd;
}

void SimplePID::setOutLimit(double out_max, double out_min)
{
  outMax = out_max;
  outMin = out_min;
}

void SimplePID::begin()
{
  reset();
}

double SimplePID::compute(double target, double input)
{
  double dt = (double)(esp_timer_get_time() - lastTime)/1000000.0;

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