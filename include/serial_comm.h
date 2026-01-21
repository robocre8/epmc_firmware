#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include "command_functions.h"

static_assert(sizeof(float) == 4, "Float must be 32-bit");

inline float readFloat(const uint8_t* data, uint8_t offset) {
  float v;
  memcpy(&v, &data[offset], sizeof(float));
  return v;
}

static inline void processCommand(uint8_t cmd, uint8_t* data) {

  gpio_set_level((gpio_num_t)LED_PIN, 1);

  bool needsFlush = false;

  switch (cmd) {
    case WRITE_VEL: {
      float v0 = readFloat(data, 0);
      float v1 = readFloat(data, 4);
      writeSpeed(v0, v1);
      break;
    }


    case WRITE_PWM: {
      float pwm0 = readFloat(data, 0);
      float pwm1 = readFloat(data, 4);
      writePWM((int)pwm0, (int)pwm1);
      break;
    }


    case READ_POS: {
      float pos0, pos1;
      readPos(pos0, pos1);

      uint8_t tx[8];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &pos0, sizeof(pos0)); tx_len += 4;
      memcpy(&tx[tx_len], &pos1, sizeof(pos1)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }


    case READ_VEL: {
      float v0, v1;
      readFilteredVel(v0, v1);

      uint8_t tx[8];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &v0, sizeof(v0)); tx_len += 4;
      memcpy(&tx[tx_len], &v1, sizeof(v1)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }


    case READ_UVEL: {
      float v0, v1;
      readUnfilteredVel(v0, v1);
      
      uint8_t tx[8];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &v0, sizeof(v0)); tx_len += 4;
      memcpy(&tx[tx_len], &v1, sizeof(v1)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }


    case READ_TVEL: {
      float v0, v1;
      readTargetVel(v0, v1);
      
      uint8_t tx[8];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &v0, sizeof(v0)); tx_len += 4;
      memcpy(&tx[tx_len], &v1, sizeof(v1)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }

    case SET_PPR: {
      uint8_t pos = data[0];
      float value = readFloat(data, 1);
      setEncoderPPR((int)pos, value);
      break;
    }

    case GET_PPR: {
      uint8_t pos = data[0];
      float res = getEncoderPPR((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }


    case SET_KP: {
      uint8_t pos = data[0];
      float value = readFloat(data, 1);
      setMotorKp((int)pos, value);
      break;
    }

    case GET_KP: {
      uint8_t pos = data[0];
      float res = getMotorKp((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }


    case SET_KI: {
      uint8_t pos = data[0];
      float value = readFloat(data, 1);
      setMotorKi((int)pos, value);
      break;
    }

    case GET_KI: {
      uint8_t pos = data[0];
      float res = getMotorKi((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }

    
    case SET_KD: {
      uint8_t pos = data[0];
      float value = readFloat(data, 1);
      setMotorKd((int)pos, value);
      break;
    }

    case GET_KD: {
      uint8_t pos = data[0];
      float res = getMotorKd((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }


    case SET_RDIR: {
      uint8_t pos = data[0];
      float value = readFloat(data, 1);
      setRdir((int)pos, value);
      break;
    }

    case GET_RDIR: {
      uint8_t pos = data[0];
      float res = getRdir((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }


    case SET_CUT_FREQ: {
      uint8_t pos = data[0];
      float value = readFloat(data, 1);
      setCutoffFreq((int)pos, value);
      break;
    }

    case GET_CUT_FREQ: {
      uint8_t pos = data[0];
      float res = getCutoffFreq((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }


    case SET_MAX_VEL: {
      uint8_t pos = data[0];
      float value = readFloat(data, 1);
      setMaxVel((int)pos, value);
      break;
    }

    case GET_MAX_VEL: {
      uint8_t pos = data[0];
      float res = getMaxVel((int)pos);
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }


    case SET_PID_MODE: {
      float value = readFloat(data, 1);
      setPidModeFunc((int)value);
      break;
    }

    case GET_PID_MODE: {
      float res = getPidModeFunc();
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }


    case SET_CMD_TIMEOUT: {
      float value = readFloat(data, 1);
      setCmdTimeout((int)value);
      break;
    }

    case GET_CMD_TIMEOUT: {
      float res = getCmdTimeout();
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }


    case SET_I2C_ADDR: {
      float value = readFloat(data, 1);
      setI2cAddress((int)value);
      break;
    }

    case GET_I2C_ADDR: {
      float res = getI2cAddress();
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }


    case RESET_PARAMS: {
      float res = triggerResetParams();
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }


    case READ_MOTOR_DATA: {
      float pos0, pos1, v0, v1;
      readPos(pos0, pos1);
      readFilteredVel(v0, v1);

      uint8_t tx[16];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &pos0, sizeof(pos0)); tx_len += 4;
      memcpy(&tx[tx_len], &pos1, sizeof(pos1)); tx_len += 4;
      memcpy(&tx[tx_len], &v0, sizeof(v0)); tx_len += 4;
      memcpy(&tx[tx_len], &v1, sizeof(v1)); tx_len += 4;

      Serial.write(tx, tx_len);
      needsFlush = true;
      break;
    }

    case CLEAR_DATA_BUFFER: {
      float res = clearDataBuffer();
      Serial.write((uint8_t*)&res, sizeof(res));
      needsFlush = true;
      break;
    }
    

    default: {
      float error = 0.0;
      Serial.write((uint8_t*)&error, sizeof(error));
      needsFlush = true;
      break;
    }
  }

  if (needsFlush) {
    Serial.flush();
  }

  gpio_set_level((gpio_num_t)LED_PIN, 0);
}












static inline void recieve_and_send_data() {
  static uint8_t state = 0;
  static uint8_t cmd, length;
  static uint8_t buffer[40];
  static uint8_t index = 0;
  static uint8_t checksum = 0;

  while (Serial.available()) {
    uint8_t b = Serial.read();

    switch (state) {
      case 0: // Wait for start
        if (b == START_BYTE) {
          state = 1;
          checksum = b;   // reset checksum correctly
        }
        break;

      case 1: // Command
        cmd = b;
        checksum += b;
        state = 2;
        break;

      case 2: // Length
        length = b;

        if (length > sizeof(buffer)) {
          state = 0;
          checksum = 0;
          break;
        }

        checksum += b;
        index = 0;
        state = (length == 0) ? 4 : 3;
        break;

      case 3: // Payload
        if (index < sizeof(buffer)) {
          buffer[index++] = b;
        }
        checksum += b;

        if (index >= length) {
          state = 4;
        }
        break;

      case 4: // Checksum
        if ((checksum & 0xFF) == b) {
          processCommand(cmd, buffer);
        } else {
          float error = 0.0f;
          Serial.write((uint8_t*)&error, sizeof(error));
          Serial.flush();
        }
        state = 0;
        break;
    }
  }
}

#endif