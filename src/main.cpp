#include <Arduino.h>
#include "command_functions.h"
#include "serial_comm.h"
#include "i2c_comm.h"
#include "driver/periph_ctrl.h"

//------------------------------------------------------------------------------//
void IRAM_ATTR readEncoder0()
{
  uint64_t currentTime = esp_timer_get_time();
  encoder[0].freqPerTick = 1000000.00 / (double)(currentTime - encoder[0].oldFreqTime);

  if (digitalRead(encoder[0].clkPin) == digitalRead(encoder[0].dirPin))
  {
    encoder[0].tickCount -= 1;
    encoder[0].frequency = -encoder[0].freqPerTick / encoder[0].pulsePerRev;
  }
  else
  {
    encoder[0].tickCount += 1;
    encoder[0].frequency = encoder[0].freqPerTick / encoder[0].pulsePerRev;
  }

  encoder[0].oldFreqTime = currentTime;
  encoder[0].checkFreqTime = currentTime;
}

void IRAM_ATTR readEncoder1()
{
  uint64_t currentTime = esp_timer_get_time();
  encoder[1].freqPerTick = 1000000.00 / (double)(currentTime - encoder[1].oldFreqTime);

  if (digitalRead(encoder[1].clkPin) == digitalRead(encoder[1].dirPin))
  {
    encoder[1].tickCount -= 1;
    encoder[1].frequency = -encoder[1].freqPerTick / encoder[1].pulsePerRev;
  }
  else
  {
    encoder[1].tickCount += 1;
    encoder[1].frequency = encoder[1].freqPerTick / encoder[1].pulsePerRev;
  }

  encoder[1].oldFreqTime = currentTime;
  encoder[1].checkFreqTime = currentTime;
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
uint64_t serialCommTime, serialCommTimeInterval = 5000;
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

  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin(i2cAddress);

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
  // if ((esp_timer_get_time() - serialCommTime) >= serialCommTimeInterval)
  // {
  //   recieve_and_send_data();
  //   serialCommTime = esp_timer_get_time();
  // }
  
  if ((esp_timer_get_time() - sensorReadTime) >= sensorReadTimeInterval)
  {
    for (int i = 0; i < num_of_motors; i += 1)
    {
      encoder[i].resetFrequency();
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
  if (abs(target[0]) < 0.001 && abs(target[1]) < 0.001)
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
  if (cmdVelTimeoutInterval > 0)
  {
    if ((esp_timer_get_time() - cmdVelTimeout) >= cmdVelTimeoutInterval)
    {
      if (pidMode == 1) writeSpeed(0.0, 0.0);
      else writePWM(0, 0);
    }
  }
}