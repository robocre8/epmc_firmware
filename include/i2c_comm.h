#ifndef I2C_COMM_H
#define I2C_COMM_H

#include <Wire.h>
#include "command_functions.h"

const uint8_t MAX_I2C_BUFFER = 32;
static uint8_t sendMsgBuffer[MAX_I2C_BUFFER];
static uint8_t sendMsgLength = 0;

// The arguments converted to integers
int i2c_cmd;
int i2c_cmd_pos;
float i2c_arg1;
float i2c_arg2;

void i2c_send_data(float data1=0.0, float data2=0.0){
  Serial.print(data1,4);
  Serial.print(' ');
  Serial.println(data2,4);
  Serial.flush();
}

/* Clear the current command parameters */
void i2c_resetCommand() {
  i2c_cmd = 0;
  i2c_cmd_pos = 0;
  i2c_arg1 = 0.0;
  i2c_arg2 = 0.0;
}

void clearSendMsgBuffer(){
  memset(sendMsgBuffer, 0, (size_t)MAX_I2C_BUFFER); 
}

// Pack float response into txBuffer
void prepareResponse2(float res0, float res1) {
  sendMsgLength = 8;
  memcpy(&sendMsgBuffer[0], &res0, sizeof(float));
  memcpy(&sendMsgBuffer[4], &res1, sizeof(float));
}

// Example command handler
void i2c_runCommand() {
  gpio_set_level((gpio_num_t)LED_PIN, 1);

  float r0 = 0.0f;
  float r1 = 0.0f;

  switch (i2c_cmd) {

    case WRITE_SPEED:
      writeSpeed(i2c_arg1, i2c_arg2);
      gpio_set_level((gpio_num_t)LED_PIN, 0);
      break;

    case READ_SPEED:
      readSpeed(r0, r1);
      memcpy(sendMsgBuffer, &r0, 4);
      memcpy(sendMsgBuffer + 4, &r1, 4);
      prepareResponse2(r0, r1);
      break;

    case WRITE_PWM:
      writePWM((int)i2c_arg1, (int)i2c_arg2);
      gpio_set_level((gpio_num_t)LED_PIN, 0);
      break;

    case READ_POS:
      readPos(r0, r1);
      memcpy(sendMsgBuffer, &r0, 4);
      memcpy(sendMsgBuffer + 4, &r1, 4);
      prepareResponse2(r0, r1);
      break;

    case GET_MAX_SPEED:
      r0 = getMaxSpeed((int)i2c_arg1);
      memcpy(sendMsgBuffer, &r0, 4);
      memcpy(sendMsgBuffer + 4, &r1, 4);
      prepareResponse2(r0, r1);
      break;
    
    case SET_PID_MODE:
      setPidModeFunc((int)i2c_arg2);
      gpio_set_level((gpio_num_t)LED_PIN, 0);
      break;

    case GET_PID_MODE:
      r0 = getPidModeFunc();
      memcpy(sendMsgBuffer, &r0, 4);
      memcpy(sendMsgBuffer + 4, &r1, 4);
      prepareResponse2(r0, r1);
      break;

    case SET_CMD_TIMEOUT:
      setCmdTimeout((int)i2c_arg2);
      gpio_set_level((gpio_num_t)LED_PIN, 0);
      break;

    case GET_CMD_TIMEOUT:
      //read Command Timeout
      r0 = getCmdTimeout();
      memcpy(sendMsgBuffer, &r0, 4);
      memcpy(sendMsgBuffer + 4, &r1, 4);
      prepareResponse2(r0, r1);
      break;

    case CLEAR:
      // clear all inintializing variables
      r0 = clearDataBuffer();
      memcpy(sendMsgBuffer, &r0, 4);
      memcpy(sendMsgBuffer + 4, &r1, 4);
      prepareResponse2(r0, r1);
      break;

    default:
      break;
  }
}




// Called when master requests data
void onRequest() {
  Wire.write(sendMsgBuffer, sendMsgLength);
  clearSendMsgBuffer();
  gpio_set_level((gpio_num_t)LED_PIN, 0);
}

// Called when master sends data
void onReceive(int numBytes)
{
  // Expect exactly 3 floats = 12 bytes
  if (numBytes != 12) {
    // Drain buffer if size is wrong
    while (Wire.available()) Wire.read();
    return;
  }

  uint8_t rxBuf[12];

  for (uint8_t i = 0; i < 12; i++) {
    rxBuf[i] = Wire.read();
  }

  // Unpack floats
  float cmd_f;
  memcpy(&cmd_f,  &rxBuf[0],  4);
  memcpy(&i2c_arg1, &rxBuf[4],  4);
  memcpy(&i2c_arg2, &rxBuf[8],  4);

  // Command as integer
  i2c_cmd = (int)cmd_f;

  // Execute command
  i2c_runCommand();
}

#endif