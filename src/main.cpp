#include <Arduino.h>
#include "command_functions.h"
#include "serial_comm.h"
#include "i2c_comm.h"
#include "driver/periph_ctrl.h"

//---------------------------------------------------------------------------------------------
// Timing variables in esp_timer_get_timeeconds
// please do not adjust any of the values as it can affect important operations
// uint64_t serialCommTime, serialCommTimeInterval = 5000;
uint64_t sensorReadTime, sensorReadTimeInterval = 2500;
uint64_t pidTime, pidTimeInterval = 10000;
//---------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------//

void IRAM_ATTR readEncoder0()
{
  if (gpio_get_level((gpio_num_t)encoder[0].clkPin) ==
    gpio_get_level((gpio_num_t)encoder[0].dirPin))
  {
    encoder[0].tickCount -= 1;
  }
  else
  {
    encoder[0].tickCount += 1;
  }
}

void IRAM_ATTR readEncoder1()
{
  if (gpio_get_level((gpio_num_t)encoder[1].clkPin) ==
    gpio_get_level((gpio_num_t)encoder[1].dirPin))
  {
    encoder[1].tickCount -= 1;
  }
  else
  {
    encoder[1].tickCount += 1;
  }
}

void encoderInit()
{
  for (int i = 0; i < num_of_motors; i += 1)
  {
    encoder[i].setPulsePerRev(enc_ppr[i]);
  }

  attachInterrupt(digitalPinToInterrupt(encoder[0].clkPin), readEncoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder[1].clkPin), readEncoder1, RISING);
}

//----------------------------------------------------------------------------------------------//

void velFilterInit()
{
  for (int i = 0; i < num_of_motors; i += 1)
  {
    velFilter[i].setCutOffFreq(cutOffFreq[i]);
    velFilter[i].setLoopFreq(1000000.0/(float)sensorReadTimeInterval);
  }
}

void pidInit()
{
  for (int i = 0; i < num_of_motors; i += 1)
  {
    pidMotor[i].setParameters(kp[i], ki[i], kd[i], outMin, outMax);
    pidMotor[i].setLoopFreq(1000000.0/(float)pidTimeInterval);
    pidMotor[i].begin();
  }
}


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
  uint64_t now_us = esp_timer_get_time();

  // Serial comm loop
  recieve_and_send_data();
  
  // Velocity reading and filtering
  if ((now_us - sensorReadTime) >= sensorReadTimeInterval)
  {
    for (int i = 0; i < num_of_motors; i += 1)
    {
      encoder[i].updateCounts();
      position[i] = encoder[i].getAngPos();
      unfilteredVel[i] = encoder[i].getAngVel();
      filteredVel[i] = velFilter[i].filter(unfilteredVel[i]);
    }
    sensorReadTime = now_us;
  }

  // PID control loop
  if ((now_us - pidTime) >= pidTimeInterval)
  {
    if (pidMode)
    {
      for (int i = 0; i < num_of_motors; i += 1)
      {
        output[i] = pidMotor[i].compute(target[i], filteredVel[i]);
        motor[i].sendPWM((int)output[i]);
      }
    }
    pidTime = now_us;
  }

  // stop motor if target is zero.
  if (abs(target[0]) < 0.1 && abs(target[1]) < 0.1)
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
    cmdVelTimeout = now_us;
    motor_is_commanded = false;
  }
  
  if (cmdVelTimeoutInterval > 0)
  {
    if ((now_us - cmdVelTimeout) >= cmdVelTimeoutInterval)
    {
      if (pidMode == 1) writeSpeed(0.0, 0.0);
      else writePWM(0, 0);
    }
  }
}