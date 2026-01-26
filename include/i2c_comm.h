// NOTE: This header must be included in exactly one source file

#ifndef I2C_COMM_H
#define I2C_COMM_H

#include <Wire.h>
#include "command_functions.h"

static const uint8_t MAX_I2C_BUFFER = 32;
static uint8_t sendMsgBuffer[MAX_I2C_BUFFER];
static uint8_t sendMsgLength = 0;

static_assert(sizeof(float) == 4, "Float must be 32-bit");

void clearSendMsgBuffer(){
  memset(sendMsgBuffer, 0, (size_t)MAX_I2C_BUFFER); 
  // for (uint8_t i=0; i< MAX_I2C_BUFFER; i+=1){
  //   sendMsgBuffer[i] = 0x00;
  // }
}
// Pack float response into txBuffer
void prepareResponse1(float res) {
  sendMsgLength = 4;
  memcpy(&sendMsgBuffer[0], &res, sizeof(float));
}

void prepareResponse4(float res0, float res1, float res2, float res3) {
  sendMsgLength = 16;
  memcpy(&sendMsgBuffer[0], &res0, sizeof(float));
  memcpy(&sendMsgBuffer[4], &res1, sizeof(float));
  memcpy(&sendMsgBuffer[8], &res2, sizeof(float));
  memcpy(&sendMsgBuffer[12], &res3, sizeof(float));
}

void prepareResponse8(float res0, float res1, float res2, float res3, float res4, float res5, float res6, float res7) {
  sendMsgLength = 32;
  memcpy(&sendMsgBuffer[0], &res0, sizeof(float));
  memcpy(&sendMsgBuffer[4], &res1, sizeof(float));
  memcpy(&sendMsgBuffer[8], &res2, sizeof(float));
  memcpy(&sendMsgBuffer[12], &res3, sizeof(float));
  memcpy(&sendMsgBuffer[16], &res4, sizeof(float));
  memcpy(&sendMsgBuffer[20], &res5, sizeof(float));
  memcpy(&sendMsgBuffer[24], &res6, sizeof(float));
  memcpy(&sendMsgBuffer[28], &res7, sizeof(float));
}

// Example command handler
void handleCommand(uint8_t cmd, uint8_t* data) {

  gpio_set_level((gpio_num_t)LED_PIN, 1);

  switch (cmd) {
    case WRITE_VEL: {
      float v0, v1, v2, v3;
      memcpy(&v0, &data[0], sizeof(float));
      memcpy(&v1, &data[4], sizeof(float));
      memcpy(&v2, &data[8], sizeof(float));
      memcpy(&v3, &data[12], sizeof(float));
      writeSpeed(v0, v1, v2, v3);
      gpio_set_level((gpio_num_t)LED_PIN, 0);
      break;
    }


    case WRITE_PWM: {
      float pwm0, pwm1, pwm2, pwm3;
      memcpy(&pwm0, &data[0], sizeof(float));
      memcpy(&pwm1, &data[4], sizeof(float));
      memcpy(&pwm2, &data[8], sizeof(float));
      memcpy(&pwm3, &data[12], sizeof(float));
      writePWM((int)pwm0, (int)pwm1, (int)pwm2, (int)pwm3);
      gpio_set_level((gpio_num_t)LED_PIN, 0);
      break;
    }

    case READ_MOTOR_DATA: {
      float pos0, pos1, pos2, pos3, v0, v1, v2, v3;
      readPos(pos0, pos1, pos2, pos3);
      readFilteredVel(v0, v1, v2, v3);
      prepareResponse8(pos0, pos1, pos2, pos3, v0, v1, v2, v3);
      break;
    }

    case READ_POS: {
      float pos0, pos1, pos2, pos3;
      readPos(pos0, pos1, pos2, pos3);
      prepareResponse4(pos0, pos1, pos2, pos3);
      break;
    }


    case READ_VEL: {
      float v0, v1, v2, v3;
      readFilteredVel(v0, v1, v2, v3);
      prepareResponse4(v0, v1, v2, v3);
      break;
    }


    case READ_UVEL: {
      float v0, v1, v2, v3;
      readUnfilteredVel(v0, v1, v2, v3);
      prepareResponse4(v0, v1, v2, v3);
      break;
    }


    case SET_CMD_TIMEOUT: {
      float value;
      memcpy(&value, &data[1], sizeof(float));
      setCmdTimeout((int)value);
      gpio_set_level((gpio_num_t)LED_PIN, 0);
      break;
    }

    case GET_CMD_TIMEOUT: {
      float res = getCmdTimeout();
      prepareResponse1(res);
      break;
    }


    case SET_PID_MODE: {
      float value;
      memcpy(&value, &data[1], sizeof(float));
      setPidModeFunc((int)value);
      gpio_set_level((gpio_num_t)LED_PIN, 0);
      break;
    }

    case GET_PID_MODE: {
      float res = getPidModeFunc();
      prepareResponse1(res);
      break;
    }

    case GET_MAX_VEL: {
      uint8_t pos = data[0];
      float res = getMaxVel((int)pos);
      prepareResponse1(res);
      break;
    }

    case CLEAR_DATA_BUFFER: {
      float res = clearDataBuffer();
      prepareResponse1(res);
      break;
    }

    default: {
      float error = 0.0;
      prepareResponse1(error);
      break;
    }
  }
  
}




// Called when master requests data
void onRequest() {
  Wire.write(sendMsgBuffer, sendMsgLength);
  clearSendMsgBuffer();
  gpio_set_level((gpio_num_t)LED_PIN, 0);
  sendMsgLength = 0;
}

// Called when master sends data
void onReceive(int numBytes) {
  static uint8_t readState = 0;
  static uint8_t msgCmd, msgLength;
  static uint8_t msgBuffer[MAX_I2C_BUFFER];
  static uint8_t msgIndex = 0;
  static uint8_t msgChecksum = 0;

  while (Wire.available()) {
    uint8_t b = Wire.read();

    switch (readState) {
      case 0: // Wait for start
        if (b == START_BYTE) {
          readState = 1;
          msgChecksum = b;
        }
        break;

      case 1: // Command
        msgCmd = b;
        msgChecksum += b;
        readState = 2;
        break;

      case 2: // Length
        msgLength = b;
        msgChecksum += b;

        if (msgLength > MAX_I2C_BUFFER) {
          readState = 0;
          msgChecksum = 0;
          break;
        }

        if (msgLength == 0) {
          readState = 4;
        } else {
          msgIndex = 0;
          readState = 3;
        }
        break;

      case 3: // Payload
        msgBuffer[msgIndex++] = b;
        msgChecksum += b;
        if (msgIndex >= msgLength) readState = 4;
        break;

      case 4: // Checksum
        if ((msgChecksum & 0xFF) == b) {
          handleCommand(msgCmd, msgBuffer);
        } else {
          float error = 0.0;
          prepareResponse1(error);
        }
  
        readState = 0; // reset for next packet
        break;
    }
  }

}

#endif