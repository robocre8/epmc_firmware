#include <Arduino.h>
#include "command_functions.h"
#include "serial_comm.h"
#include "i2c_comm.h"
#include "driver/periph_ctrl.h"

//------------------------------------------------------------------------------//

void IRAM_ATTR readEncoder0()
{
  uint64_t currentPeriodCount = esp_timer_get_time();

  if (gpio_get_level((gpio_num_t)encoder[0].clkPin) ==
    gpio_get_level((gpio_num_t)encoder[0].dirPin))
  {
    encoder[0].tickCount -= 1;
    encoder[0].dir = -1;
  }
  else
  {
    encoder[0].tickCount += 1;
    encoder[0].dir = 1;
  }

  encoder[0].prevPeriodCount = encoder[0].periodCount;
  encoder[0].periodCount = currentPeriodCount;
  encoder[0].checkPeriodCount = currentPeriodCount;
}

void IRAM_ATTR readEncoder1()
{
  uint64_t currentPeriodCount = esp_timer_get_time();

  if (gpio_get_level((gpio_num_t)encoder[1].clkPin) ==
    gpio_get_level((gpio_num_t)encoder[1].dirPin))
  {
    encoder[1].tickCount -= 1;
    encoder[1].dir = -1;
  }
  else
  {
    encoder[1].tickCount += 1;
    encoder[1].dir = 1;
  }

  encoder[1].prevPeriodCount = encoder[1].periodCount;
  encoder[1].periodCount = currentPeriodCount;
  encoder[1].checkPeriodCount = currentPeriodCount;
}

void IRAM_ATTR readEncoder2()
{
  uint64_t currentPeriodCount = esp_timer_get_time();

  if (gpio_get_level((gpio_num_t)encoder[2].clkPin) ==
    gpio_get_level((gpio_num_t)encoder[2].dirPin))
  {
    encoder[2].tickCount -= 1;
    encoder[2].dir = -1;
  }
  else
  {
    encoder[2].tickCount += 1;
    encoder[2].dir = 1;
  }

  encoder[2].prevPeriodCount = encoder[2].periodCount;
  encoder[2].periodCount = currentPeriodCount;
  encoder[2].checkPeriodCount = currentPeriodCount;
}

void IRAM_ATTR readEncoder3()
{
  uint64_t currentPeriodCount = esp_timer_get_time();

  if (gpio_get_level((gpio_num_t)encoder[3].clkPin) ==
    gpio_get_level((gpio_num_t)encoder[3].dirPin))
  {
    encoder[3].tickCount -= 1;
    encoder[3].dir = -1;
  }
  else
  {
    encoder[3].tickCount += 1;
    encoder[3].dir = 1;
  }

  encoder[3].prevPeriodCount = encoder[3].periodCount;
  encoder[3].periodCount = currentPeriodCount;
  encoder[3].checkPeriodCount = currentPeriodCount;
}

void encoderInit()
{
  for (int i = 0; i < num_of_motors; i += 1)
  {
    encoder[i].setPulsePerRev(enc_ppr[i]);
  }

  attachInterrupt(digitalPinToInterrupt(encoder[0].clkPin), readEncoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder[1].clkPin), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder[2].clkPin), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder[3].clkPin), readEncoder3, RISING);
}

//----------------------------------------------------------------------------------------------//

void velFilterInit()
{
  for (int i = 0; i < num_of_motors; i += 1)
  {
    velFilter[i].setCutOffFreq(cutOffFreq[i]);
  }
}

void pidInit()
{
  for (int i = 0; i < num_of_motors; i += 1)
  {
    pidMotor[i].setParameters(kp[i], ki[i], kd[i], outMin, outMax);
    pidMotor[i].begin();
  }
}

//---------------------------------------------------------------------------------------------
// Timing variables in esp_timer_get_timeeconds
// please do not adjust any of the values as it can affect important operations
// uint64_t serialCommTime, serialCommTimeInterval = 5000;
uint64_t sensorReadTime, sensorReadTimeInterval = 1000;
uint64_t pidTime, pidTimeInterval = 2500;
//---------------------------------------------------------------------------------------------


void setup()
{
  loadStoredParams();

  // Serial.begin(57600);
  Serial.begin(115200);
  // Serial.begin(460800);
  // Serial.begin(921600);
  // Serial.setTimeout(2);

  Wire.begin(i2cAddress);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  analogWriteResolution(8); // 8 Bit resolution
  analogWriteFrequency(1000); // 1kHz
  // analogWriteFrequency(5000); // 5kHz
  // analogWriteFrequency(10000); // 10kHz

  encoderInit();
  velFilterInit();
  pidInit();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);

  // Initialize timing markers
  uint64_t now_us = esp_timer_get_time();
  sensorReadTime = now_us;
  pidTime = now_us;
  cmdVelTimeout = now_us;

}

void loop()
{
  // Serial comm loop
  recieve_and_send_data();
  
  // Velocity reading and filtering
  if ((esp_timer_get_time() - sensorReadTime) >= sensorReadTimeInterval)
  {
    for (int i = 0; i < num_of_motors; i += 1)
    {
      encoder[i].resetFrequency(); // ensures readinding of zero velocity when motor stops
      unfilteredVel[i] = encoder[i].getAngVel();
      filteredVel[i] = velFilter[i].filter(unfilteredVel[i]);
    }
    sensorReadTime = esp_timer_get_time();
  }

  // PID control loop
  if ((esp_timer_get_time() - pidTime) >= pidTimeInterval)
  {
    if (pidMode)
    {
      for (int i = 0; i < num_of_motors; i += 1)
      {
        output[i] = pidMotor[i].compute(target[i], filteredVel[i]);
        motor[i].sendPWM((int)output[i]);
      }
    }
    pidTime = esp_timer_get_time();
  }

  // stop motor if target is zero.
  if (abs(target[0]) < 0.001 && abs(target[1]) < 0.001 && abs(target[2]) < 0.001 && abs(target[3]) < 0.001)
  {
    if (pidMode == 1)
    {
        for (int i = 0; i < num_of_motors; i += 1)
        {
          target[i] = 0.0;
          pidMotor[i].reset();
        }
        setPidModeFunc(0);
    }
  }
  else
  {
    if (pidMode == 0)
    {
      setPidModeFunc(1);
    }
  }

  // command timeout
  if (motor_is_commanded) {
    cmdVelTimeout = esp_timer_get_time();
    motor_is_commanded = false;
  }
  
  if (cmdVelTimeoutInterval > 0)
  {
    if ((esp_timer_get_time() - cmdVelTimeout) >= cmdVelTimeoutInterval)
    {
      if (pidMode == 1) writeSpeed(0.0, 0.0, 0.0, 0.0);
      else writePWM(0, 0, 0, 0);
    }
  }
}