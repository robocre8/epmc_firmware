#include "l298n_motor_control.h"

MotorControl::MotorControl(int IN1_pin, int IN2_pin, int en_pin)
{
  in1Pin = IN1_pin;
  in2Pin = IN2_pin;
  enPin = en_pin;

  use_enPin = true;

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enPin, OUTPUT);

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
}

MotorControl::MotorControl(int IN1_pin, int IN2_pin)
{
  in1Pin = IN1_pin;
  in2Pin = IN2_pin;

  use_enPin = false;

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  analogWrite(in1Pin, 0);
  analogWrite(in2Pin, 0);
}

void MotorControl::sendPWM(int pwmVal)
{
  if (use_enPin) {
    if (pwmVal > 0)
    {
      analogWrite(enPin, abs(pwmVal));
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
    }
    else if (pwmVal < 0)
    {
      analogWrite(enPin, abs(pwmVal));
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
    }
    else
    {
      analogWrite(enPin, 0);
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, LOW);
    }
  }
  else {
    if (pwmVal > 0)
    {
      analogWrite(in1Pin, abs(pwmVal));
      analogWrite(in2Pin, LOW);
    }
    else if (pwmVal < 0)
    {
      analogWrite(in1Pin, LOW);
      analogWrite(in2Pin, abs(pwmVal));
    }
    else
    {
      analogWrite(in1Pin, 0);
      analogWrite(in2Pin, 0);
    }
  }
}