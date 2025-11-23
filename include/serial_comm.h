#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include "command_functions.h"


void processCommand(uint8_t cmd, uint8_t* data, uint8_t length) {

  digitalWrite(LED_PIN, HIGH);
  switch (cmd) {
    case WRITE_VEL: {
      float v0, v1;
      memcpy(&v0, &data[0], sizeof(float));
      memcpy(&v1, &data[4], sizeof(float));
      writeSpeed(v0, v1);
      break;
    }


    case WRITE_PWM: {
      float pwm0, pwm1;
      memcpy(&pwm0, &data[0], sizeof(float));
      memcpy(&pwm1, &data[4], sizeof(float));
      writePWM((int)pwm0, (int)pwm1);
      break;
    }


    case READ_POS: {
      float pos0, pos1;
      readPos(pos0, pos1);
      Serial.write((uint8_t*)&pos0, sizeof(pos0));
      Serial.write((uint8_t*)&pos1, sizeof(pos1));
      break;
    }


    case READ_VEL: {
      float v0, v1;
      readFilteredVel(v0, v1);
      Serial.write((uint8_t*)&v0, sizeof(v0));
      Serial.write((uint8_t*)&v1, sizeof(v1));
      break;
    }


    case READ_UVEL: {
      float v0, v1;
      readUnfilteredVel(v0, v1);
      Serial.write((uint8_t*)&v0, sizeof(v0));
      Serial.write((uint8_t*)&v1, sizeof(v1));
      break;
    }


    case READ_TVEL: {
      float v0, v1;
      readTargetVel(v0, v1);
      Serial.write((uint8_t*)&v0, sizeof(v0));
      Serial.write((uint8_t*)&v1, sizeof(v1));
      break;
    }

    case SET_PPR: {
      uint8_t pos = data[0];
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setEncoderPPR((int)pos, (double)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_PPR: {
      uint8_t pos = data[0];
      float res = getEncoderPPR((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_KP: {
      uint8_t pos = data[0];
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setMotorKp((int)pos, (double)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_KP: {
      uint8_t pos = data[0];
      float res = getMotorKp((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_KI: {
      uint8_t pos = data[0];
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setMotorKi((int)pos, (double)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_KI: {
      uint8_t pos = data[0];
      float res = getMotorKi((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    
    case SET_KD: {
      uint8_t pos = data[0];
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setMotorKd((int)pos, (double)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_KD: {
      uint8_t pos = data[0];
      float res = getMotorKd((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_RDIR: {
      uint8_t pos = data[0];
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setRdir((int)pos, (float)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_RDIR: {
      uint8_t pos = data[0];
      float res = getRdir((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_CUT_FREQ: {
      uint8_t pos = data[0];
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setCutoffFreq((int)pos, (double)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_CUT_FREQ: {
      uint8_t pos = data[0];
      float res = getCutoffFreq((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_MAX_VEL: {
      uint8_t pos = data[0];
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setMaxVel((int)pos, (double)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_MAX_VEL: {
      uint8_t pos = data[0];
      float res = getMaxVel((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_PID_MODE: {
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setPidModeFunc((int)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_PID_MODE: {
      float res = getPidModeFunc();
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_CMD_TIMEOUT: {
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setCmdTimeout((int)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_CMD_TIMEOUT: {
      float res = getCmdTimeout();
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case SET_I2C_ADDR: {
      float value;
      memcpy(&value, &data[1], sizeof(float));
      float res = setI2cAddress((int)value);
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }

    case GET_I2C_ADDR: {
      float res = getI2cAddress();
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case RESET_PARAMS: {
      float res = triggerResetParams();
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }


    case READ_MOTOR_DATA: {
      float pos0, pos1, v0, v1;
      readPos(pos0, pos1);
      readFilteredVel(v0, v1);
      Serial.write((uint8_t*)&pos0, sizeof(pos0));
      Serial.write((uint8_t*)&pos1, sizeof(pos1));
      Serial.write((uint8_t*)&v0, sizeof(v0));
      Serial.write((uint8_t*)&v1, sizeof(v1));
      break;
    }

    case CLEAR_DATA_BUFFER: {
      float res = clearDataBuffer();
      Serial.write((uint8_t*)&res, sizeof(res));
      break;
    }
    

    default: {
      float error = 0.0;
      Serial.write((uint8_t*)&error, sizeof(error));
      break;
    }
  }

  digitalWrite(LED_PIN, LOW);
}












void recieve_and_send_data() {
  static uint8_t state = 0;
  static uint8_t cmd, length;
  static uint8_t buffer[32];
  static uint8_t index = 0;
  static uint8_t checksum = 0;

  while (Serial.available()) {
    uint8_t b = Serial.read();

    switch (state) {
      case 0: // Wait for start
        if (b == START_BYTE) {
          state = 1;
          checksum = b;
        }
        break;

      case 1: // Command
        cmd = b;
        checksum += b;
        state = 2;
        break;

      case 2: // Length
        length = b;
        checksum += b;
        if (length==0){
          state = 4;
        }
        else{
          index = 0;
          state = 3;
        }
        break;

      case 3: // Payload
        buffer[index++] = b;
        checksum += b;
        if (index >= length) state = 4;
        break;

      case 4: // Checksum
        if ((checksum & 0xFF) == b) {
          processCommand(cmd, buffer, length);
        } else {
          float error = 0.0;
          Serial.write((uint8_t*)&error, sizeof(error));
        }
        state = 0; // reset for next packet
        break;
    }
  }
}

#endif