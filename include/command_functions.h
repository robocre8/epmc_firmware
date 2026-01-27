#ifndef COMMAND_FUNCTIONS_H
#define COMMAND_FUNCTIONS_H

#include <Arduino.h>
#include <Preferences.h>
#include "l298n_motor_control.h"
#include "encoder_setup.h"
#include "adaptive_low_pass_filter.h"
#include "simple_pid_control.h"

//------------ Communication Command IDs --------------//
enum CommandID : uint8_t {
  START_BYTE = 0xAA,
  WRITE_VEL = 0x01,
  WRITE_PWM = 0x02,
  READ_POS = 0x03,
  READ_VEL = 0x04,
  READ_UVEL = 0x05,
  READ_TVEL = 0x06,
  SET_PPR = 0x07,
  GET_PPR = 0x08,
  SET_KP = 0x09,
  GET_KP = 0x0A,
  SET_KI = 0x0B,
  GET_KI = 0x0C,
  SET_KD = 0x0D,
  GET_KD = 0x0E,
  SET_RDIR = 0x0F,
  GET_RDIR = 0x10,
  SET_CUT_FREQ = 0x11,
  GET_CUT_FREQ = 0x12,
  SET_MAX_VEL = 0x13,
  GET_MAX_VEL = 0x14,
  SET_PID_MODE = 0x15,
  GET_PID_MODE = 0x16,
  SET_CMD_TIMEOUT = 0x17,
  GET_CMD_TIMEOUT = 0x18,
  SET_I2C_ADDR = 0x19,
  GET_I2C_ADDR = 0x1A,
  RESET_PARAMS = 0x1B,
  READ_MOTOR_DATA = 0x2A,
  CLEAR_DATA_BUFFER = 0x2C,
};
//---------------------------------------------------//

//--------------- global variables -----------------//
const int LED_PIN = 2;

const int num_of_motors = 4;

// motor 0 H-Bridge Connection
int IN1_0 = 5, IN2_0 = 17;
// motor 1 H-Bridge Connection
int IN1_1 = 19, IN2_1 = 18;
// motor 2 H-Bridge Connection
int IN1_2 = 26, IN2_2 = 27;
// motor 3 H-Bridge Connection
int IN1_3 = 33, IN2_3 = 25;

MotorControl motor[num_of_motors] = {
  MotorControl(IN1_0, IN2_0), // motor 0
  MotorControl(IN1_1, IN2_1), // motor 1
  MotorControl(IN1_2, IN2_2), // motor 2
  MotorControl(IN1_3, IN2_3), // motor 3
};

float enc_ppr[num_of_motors]={
  1000.0, // motor 0 encoder pulse per revolution parameter
  1000.0, // motor 1 encoder pulse per revolution parameter
  1000.0, // motor 2 encoder pulse per revolution parameter
  1000.0, // motor 3 encoder pulse per revolution parameter
};

// motor 0 encoder connection
int enc0_clkPin = 15, enc0_dirPin = 4;
// motor 1 encoder connection
int enc1_clkPin = 35, enc1_dirPin = 34;
// motor 2 encoder connection
int enc2_clkPin = 13, enc2_dirPin = 14;
// motor 3 encoder connection
int enc3_clkPin = 39, enc3_dirPin = 36;

QuadEncoder encoder[num_of_motors] = {
  QuadEncoder(enc0_clkPin, enc0_dirPin, enc_ppr[0]), // motor 0 encoder connection
  QuadEncoder(enc1_clkPin, enc1_dirPin, enc_ppr[1]), // motor 1 encoder connection
  QuadEncoder(enc2_clkPin, enc2_dirPin, enc_ppr[2]), // motor 2 encoder connection
  QuadEncoder(enc3_clkPin, enc3_dirPin, enc_ppr[3]), // motor 3 encoder connection
};

// adaptive lowpass Filter
const int filterOrder = 1;
float cutOffFreq[num_of_motors] = {
  2.5, // motor 0 velocity filter cutoff frequency
  2.5, // motor 1 velocity filter cutoff frequency
  2.5, // motor 2 velocity filter cutoff frequency
  2.5, // motor 3 velocity filter cutoff frequency
};

AdaptiveLowPassFilter velFilter[num_of_motors] = {
  AdaptiveLowPassFilter(filterOrder, cutOffFreq[0]), // motor 0 velocity filter
  AdaptiveLowPassFilter(filterOrder, cutOffFreq[1]), // motor 1 velocity filter
  AdaptiveLowPassFilter(filterOrder, cutOffFreq[2]), // motor 2 velocity filter
  AdaptiveLowPassFilter(filterOrder, cutOffFreq[3]), // motor 3 velocity filter
};

float filteredVel[num_of_motors] = {
  0.0,
  0.0,
  0.0,
  0.0,
};

float unfilteredVel[num_of_motors] = {
  0.0,
  0.0,
  0.0,
  0.0,
};


// motor PID parameters
float outMin = -255.0, outMax = 255.0;

float kp[num_of_motors] = {
  0.0,
  0.0,
  0.0,
  0.0,
};

float ki[num_of_motors] = {
  0.0,
  0.0,
  0.0,
  0.0,
};

float kd[num_of_motors] = {
  0.0,
  0.0,
  0.0,
  0.0,
};

float target[num_of_motors] = {
  0.0,
  0.0,
  0.0,
  0.0,
};

float output[num_of_motors] = {
  0.0,
  0.0,
  0.0,
  0.0,
};

SimplePID pidMotor[num_of_motors] = {
  SimplePID(kp[0], ki[0], kd[0], outMin, outMax),
  SimplePID(kp[1], ki[1], kd[1], outMin, outMax),
  SimplePID(kp[2], ki[2], kd[2], outMin, outMax),
  SimplePID(kp[3], ki[3], kd[3], outMin, outMax),
};


// check if in PID or PWM mode
int pidMode = 0;

int rdir[num_of_motors] = {
  1,
  1,
  1,
  1,
};

// // maximum motor velocity that can be commanded
float maxVel[num_of_motors] = {
  10.0,
  10.0,
  10.0,
  10.0,
};

// for command timeout.
uint64_t cmdVelTimeoutInterval = 0; // us -> (1000000/sampleTime) hz
uint64_t cmdVelTimeout;
bool motor_is_commanded = false;

// initial i2cAddress
uint8_t i2cAddress = 0x55;

// for stored initialization and reset
bool firstLoad = false;
//-------------------------------------------------//


//--------------- storage variables -----------------//
Preferences storage;

const char * ppr_key[num_of_motors] = {
  "ppr0",
  "ppr1",
  "ppr2",
  "ppr3",
};

const char * cf_key[num_of_motors] = {
  "cf0",
  "cf1",
  "cf2",
  "cf3",
};

const char * kp_key[num_of_motors] = {
  "kp0",
  "kp1",
  "kp2",
  "kp3",
};

const char * ki_key[num_of_motors] = {
  "ki0",
  "ki1",
  "ki2",
  "ki3",
};

const char * kd_key[num_of_motors] = {
  "kd0",
  "kd1",
  "kd2",
  "kd3",
};

const char * rdir_key[num_of_motors] = {
  "rdir0",
  "rdir1",
  "rdir2",
  "rdir3",
};

const char * maxVel_key[num_of_motors] = {
  "maxVel0",
  "maxVel1",
  "maxVel2",
  "maxVel3",
};

const char * i2cAddress_key = "i2cAddress";

const char * firstLoad_key = "firstLoad";

const char * params_ns = "params"; // preference namespace

void resetParamsInStorage(){
  storage.begin(params_ns, false);

  for (int i=0; i<num_of_motors; i+=1){
    storage.putFloat(ppr_key[i], 1000.0);
    storage.putFloat(kp_key[i], 0.0);
    storage.putFloat(ki_key[i], 0.0);
    storage.putFloat(kd_key[i], 0.0);
    storage.putFloat(cf_key[i], 2.5);
    storage.putInt(rdir_key[i], 1);
    storage.putFloat(maxVel_key[i], 10.0);
  }
  storage.putUChar(i2cAddress_key, 0x55);

  storage.end();
}

void initParams(){
  //check for firstLoad
  storage.begin(params_ns, true);
  firstLoad = storage.getBool(firstLoad_key);
  storage.end();
  // if firsLoad -> reset all params and set firstLoad to false
  if(firstLoad == true){
    resetParamsInStorage();
    firstLoad = false;
    storage.begin(params_ns, false);
    storage.putBool(firstLoad_key, firstLoad);
    storage.end();
  }

}

void loadStoredParams(){
  initParams();
  // load each parameter form the storage to the local variables
  storage.begin(params_ns, true);

  for (int i=0; i<num_of_motors; i+=1){
    enc_ppr[i] = storage.getFloat(ppr_key[i], 1000.0);
    kp[i] = storage.getFloat(kp_key[i], 0.0);
    ki[i] = storage.getFloat(ki_key[i], 0.0);
    kd[i] = storage.getFloat(kd_key[i], 0.0);
    cutOffFreq[i] = storage.getFloat(cf_key[i], 2.5);
    rdir[i] = storage.getInt(rdir_key[i], 1);
    maxVel[i] = storage.getFloat(maxVel_key[i], 10.0);
  }
  i2cAddress = storage.getUChar(i2cAddress_key, 0x55);

  storage.end();
}
//-------------------------------------------------//




//--------------- global functions ----------------//
float writeSpeed(float v0, float v1, float v2, float v3)
{
  float targetVel[num_of_motors] = {v0, v1, v2, v3};
  for (int i = 0; i < num_of_motors; i += 1)
  {
    float tVel = constrain(targetVel[i], -1.00 * maxVel[i], maxVel[i]);
    target[i] = (float)rdir[i] * tVel;
  }
  motor_is_commanded = true;
  // cmdVelTimeout = esp_timer_get_time();

  return 1.0;
}

float writePWM(int pwm0, int pwm1, int pwm2, int pwm3)
{
  int pwm[num_of_motors] = {pwm0, pwm1, pwm2, pwm3};
  if(pidMode == 0){
    for (int i = 0; i < num_of_motors; i += 1){
      int p = constrain(pwm[i], -255, 255);
      motor[i].sendPWM(rdir[i] * p);
    }
    motor_is_commanded = true;
    // cmdVelTimeout = esp_timer_get_time();
  }
  
  return 1.0;
}

void readPos(float &pos0, float &pos1, float &pos2, float &pos3)
{  
  float posData[num_of_motors];
  for (int i = 0; i < num_of_motors; i += 1){
    posData[i] = rdir[i] * encoder[i].getAngPos();
  }
  pos0 = (float)posData[0];
  pos1 = (float)posData[1];
  pos2 = (float)posData[2];
  pos3 = (float)posData[3];
}

void readFilteredVel(float &v0, float &v1, float &v2, float &v3)
{
  float velData[num_of_motors];
  for (int i = 0; i < num_of_motors; i += 1){
    velData[i] = (float)rdir[i] * filteredVel[i];
  }
  v0 = (float)velData[0];
  v1 = (float)velData[1];
  v2 = (float)velData[2];
  v3 = (float)velData[3];
}

void readUnfilteredVel(float &v0, float &v1, float &v2, float &v3)
{
  float velData[num_of_motors];
  for (int i = 0; i < num_of_motors; i += 1){
    velData[i] = (float)rdir[i] * unfilteredVel[i];
  }
  v0 = (float)velData[0];
  v1 = (float)velData[1];
  v2 = (float)velData[2];
  v3 = (float)velData[3];
}

void readTargetVel(float &v0, float &v1, float &v2, float &v3)
{
  float velData[num_of_motors];
  for (int i = 0; i < num_of_motors; i += 1){
    velData[i] = (float)rdir[i] * target[i];
  }
  v0 = (float)velData[0];
  v1 = (float)velData[1];
  v2 = (float)velData[2];
  v3 = (float)velData[3];
}

float setEncoderPPR(int motor_no, float ppr)
{
  if (motor_no<0 || motor_no>num_of_motors-1)
    return 0.0;

  enc_ppr[motor_no] = ppr;
  storage.begin(params_ns, false);
  storage.putFloat(ppr_key[motor_no], enc_ppr[motor_no]);
  storage.end();
  encoder[motor_no].setPulsePerRev(enc_ppr[motor_no]);
  return 1.0;
}
float getEncoderPPR(int motor_no)
{
  if (motor_no<0 || motor_no>num_of_motors-1)
    return 1000.0;

  return (float)enc_ppr[motor_no];
}


float setMotorKp(int motor_no, float Kp)
{
  if (motor_no<0 || motor_no>num_of_motors-1)
    return 0.0;

  kp[motor_no] = Kp;
  storage.begin(params_ns, false);
  storage.putFloat(kp_key[motor_no], kp[motor_no]);
  storage.end();
  pidMotor[motor_no].setKp(kp[motor_no]);
  pidMotor[motor_no].begin();
  return 1.0;
}
float getMotorKp(int motor_no)
{
  if (motor_no<0 || motor_no>num_of_motors-1)
    return 0.0;

  return (float)kp[motor_no];
}


float setMotorKi(int motor_no, float Ki)
{
  if (motor_no<0 || motor_no>num_of_motors-1)
    return 0.0;

  ki[motor_no] = Ki;
  storage.begin(params_ns, false);
  storage.putFloat(ki_key[motor_no], ki[motor_no]);
  storage.end();
  pidMotor[motor_no].setKi(ki[motor_no]);
  pidMotor[motor_no].begin();
  return 1.0;
}
float getMotorKi(int motor_no)
{
  if (motor_no<0 || motor_no>num_of_motors-1)
    return 0.0;

  return (float)ki[motor_no];
}


float setMotorKd(int motor_no, float Kd)
{
  if (motor_no<0 || motor_no>num_of_motors-1)
    return 0.0;

  kd[motor_no] = Kd;
  storage.begin(params_ns, false);
  storage.putFloat(kd_key[motor_no], kd[motor_no]);
  storage.end();
  pidMotor[motor_no].setKd(kd[motor_no]);
  pidMotor[motor_no].begin();
  return 1.0;
}
float getMotorKd(int motor_no)
{
  if (motor_no<0 || motor_no>num_of_motors-1)
    return 0.0;

  return (float)kd[motor_no];
}


float setRdir(int motor_no, float dir)
{
  if (motor_no<0 || motor_no>num_of_motors-1)
    return 0.0;

  if (dir >= 0)
    rdir[motor_no] = 1;
  else
    rdir[motor_no] = -1;
  storage.begin(params_ns, false);
  storage.putInt(rdir_key[motor_no], rdir[motor_no]);
  storage.end();
  return 1.0;
}
float getRdir(int motor_no)
{
  if (motor_no<0 || motor_no>num_of_motors-1)
    return 1.0;

  return (float)rdir[motor_no];
}



float setCutoffFreq(int motor_no, float f0)
{
  if (motor_no<0 || motor_no>num_of_motors-1)
    return 0.0;

  cutOffFreq[motor_no] = f0;
  storage.begin(params_ns, false);
  storage.putFloat(cf_key[motor_no], cutOffFreq[motor_no]);
  storage.end();
  velFilter[motor_no].setCutOffFreq(cutOffFreq[motor_no]);
  return 1.0;
}
float getCutoffFreq(int motor_no)
{
  if (motor_no<0 || motor_no>num_of_motors-1)
    return 2.5;

  return (float)cutOffFreq[motor_no];
}



float setMaxVel(int motor_no, float max_vel)
{
  if (motor_no<0 || motor_no>num_of_motors-1)
    return 0.0;

  maxVel[motor_no] = fabs(max_vel);
  storage.begin(params_ns, false);
  storage.putFloat(maxVel_key[motor_no], maxVel[motor_no]);
  storage.end();
  return 1.0;
}
float getMaxVel(int motor_no)
{
  if (motor_no<0 || motor_no>num_of_motors-1)
    return 10.0;

  return (float)maxVel[motor_no];
}



float setPidModeFunc(int mode)
{
  if (mode == 0)
  {
    pidMode = 0;
    for (int i = 0; i < num_of_motors; i += 1){
      motor[i].sendPWM(0);
      pidMotor[i].begin();
    }
    return 1.0;
  }
  else if (mode == 1)
  {
    pidMode = 1;
    for (int i = 0; i < num_of_motors; i += 1){
      motor[i].sendPWM(0);
      pidMotor[i].begin();
    }
    return 1.0;
  }
  else
  {
    return 0.0;
  }
}
float getPidModeFunc()
{
  if (pidMode == 1) return 1.0;
  else return 0.0;
}


float setCmdTimeout(int timeout_ms)
{
  uint64_t cmdTimeout = timeout_ms;
  if (cmdTimeout < 100)
  {
    cmdVelTimeoutInterval = 0;
  }
  else
  {
    cmdVelTimeoutInterval = cmdTimeout*1000;
  }
  cmdVelTimeout = esp_timer_get_time();
  return 1.0;
}
float getCmdTimeout()
{
  return (float)(cmdVelTimeoutInterval/1000);
}


float clearDataBuffer()
{
  for (int i = 0; i < num_of_motors; i += 1)
  {
    encoder[i].clearTickCounts();
    filteredVel[i] = 0.0;
    unfilteredVel[i] = 0.0;
    target[i] = 0.0;
    velFilter[i].clear();
    pidMotor[i].begin();
  }
  return 1.0;
}


float triggerResetParams()
{
  storage.begin(params_ns, false);
  firstLoad = true;
  storage.putBool(firstLoad_key, firstLoad);
  storage.end();
  // reload to reset
  loadStoredParams();
  return 1.0;
}




#include "i2c_comm.h"
float setI2cAddress(int address)
{
  if((address <= 0) || (address > 255)){
    return 0.0;
  }
  else {
    i2cAddress = (uint8_t)address;
    storage.begin(params_ns, false);
    storage.putUChar(i2cAddress_key, i2cAddress);
    storage.end();

    Wire.begin(i2cAddress);

    return 1.0;
  }
}
float getI2cAddress()
{
  return (float)i2cAddress;
}
//-------------------------------------------------------------------//


#endif