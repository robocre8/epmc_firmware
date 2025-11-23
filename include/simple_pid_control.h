#ifndef SIMPLE_PID_CONTROL_H
#define SIMPLE_PID_CONTROL_H
#include <Arduino.h>


class SimplePID {
  public:
    SimplePID(double Kp, double Ki, double Kd, double out_min, double out_max);

    void setParameters(double Kp, double Ki, double Kd, double out_min, double out_max);
    void setGains(double Kp, double Ki, double Kd);
    void setKp(double Kp);
    void setKi(double Ki);
    void setKd(double Kd);
    void setOutLimit(double out_max, double out_min);
    void begin();
    void reset();
    double compute(double target, double actual);

  private:
    double p_error, d_error, i_term, prevInput, prevTarget;
    uint64_t lastTime;
    double kp, ki, kd;
    double outMax, outMin, output;
};

#endif
