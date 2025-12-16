#ifndef I2C_COMM_H
#define I2C_COMM_H

#include <Wire.h>
#include "command_functions.h"

const uint8_t MAX_I2C_BUFFER = 32;
static uint8_t sendMsgBuffer[MAX_I2C_BUFFER];
static uint8_t sendMsgLength = 0;

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

void prepareResponse2(float res0, float res1) {
  sendMsgLength = 8;
  memcpy(&sendMsgBuffer[0], &res0, sizeof(float));
  memcpy(&sendMsgBuffer[4], &res1, sizeof(float));
}

void prepareResponse4(float res0, float res1, float res2, float res3) {
  sendMsgLength = 16;
  memcpy(&sendMsgBuffer[0], &res0, sizeof(float));
  memcpy(&sendMsgBuffer[4], &res1, sizeof(float));
  memcpy(&sendMsgBuffer[8], &res2, sizeof(float));
  memcpy(&sendMsgBuffer[12], &res3, sizeof(float));
}

// Example command handler
void handleCommand(uint8_t cmd, uint8_t* data) {

  gpio_set_level((gpio_num_t)LED_PIN, 1);

  switch (cmd) {
    case WRITE_VEL: {
      float v0, v1;
      memcpy(&v0, &data[0], sizeof(float));
      memcpy(&v1, &data[4], sizeof(float));
      writeSpeed(v0, v1);
      gpio_set_level((gpio_num_t)LED_PIN, 0);
      break;
    }


    case WRITE_PWM: {
      float pwm0, pwm1;
      memcpy(&pwm0, &data[0], sizeof(float));
      memcpy(&pwm1, &data[4], sizeof(float));
      writePWM((int)pwm0, (int)pwm1);
      gpio_set_level((gpio_num_t)LED_PIN, 0);
      break;
    }

    case READ_MOTOR_DATA: {
      float pos0, pos1, v0, v1;
      readPos(pos0, pos1);
      readFilteredVel(v0, v1);
      prepareResponse4(pos0, pos1, v0, v1);
      break;
    }

    case READ_POS: {
      float pos0, pos1;
      readPos(pos0, pos1);
      prepareResponse2(pos0, pos1);
      break;
    }


    case READ_VEL: {
      float v0, v1;
      readFilteredVel(v0, v1);
      prepareResponse2(v0, v1);
      break;
    }


    case READ_UVEL: {
      float v0, v1;
      readUnfilteredVel(v0, v1);
      prepareResponse2(v0, v1);
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
        if (msgLength==0){
          readState = 4;
        }
        else{
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
          switch(msgLength){
            case 4: {
              prepareResponse1(error);
              break;
            }
            case 8: {
              prepareResponse2(error, error);
              break;
            }
            case 16: {
              prepareResponse4(error, error, error, error);
              break;
            }
            default: {
              prepareResponse1(error);
              break;
            }
          }
          
        }
        readState = 0; // reset for next packet
        break;
    }
  }

}

#endif