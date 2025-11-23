#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <Arduino.h>

class MotorControl {
  public:
    MotorControl(int IN1_pin, int IN2_pin, int en_pin);
    MotorControl(int IN1_pin, int IN2_pin);
    void sendPWM(int pwmVal);

  private:
    int in1Pin, in2Pin, enPin;
    bool use_enPin = true;

};

#endif