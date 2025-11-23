#include <Arduino.h>

// MOTOR 1
#define IN1 4
#define IN2 10

// MOTOT 2
#define IN3 0
#define IN4 1


void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  analogWrite(IN1, 0);
  analogWrite(IN2, 0);

  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

void loop() {
  delay(2000);

  analogWrite(IN1, 100);
  analogWrite(IN2, 0);

  analogWrite(IN3, 100);
  analogWrite(IN4, 0);

  delay(4000);

  analogWrite(IN1, 0);
  analogWrite(IN2, 0);

  analogWrite(IN3, 0);
  analogWrite(IN4, 0);

  delay(2000);

  analogWrite(IN1, 0);
  analogWrite(IN2, 200);

  analogWrite(IN3, 0);
  analogWrite(IN4, 200);

  delay(4000);

  analogWrite(IN1, 0);
  analogWrite(IN2, 0);

  analogWrite(IN3, 0);
  analogWrite(IN4, 0);

}