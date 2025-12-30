#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include "command_functions.h"

// The arguments converted to integers
int cmd;
int cmd_pos;
float arg1;
float arg2;
char null_char = '\0';

void send_data(float data1=0.0, float data2=0.0){
  Serial.print(data1,4);
  Serial.print(' ');
  Serial.println(data2,4);
  Serial.flush();
}

/* Clear the current command parameters */
void resetCommand() {
  cmd = 0;
  cmd_pos = 0;
  arg1 = 0.0;
  arg2 = 0.0;
}

/* Run a command.  Commands are defined in commands.h */
void runCommand() {
  gpio_set_level((gpio_num_t)LED_PIN, 1);

  switch (cmd) {
    case WRITE_SPEED: {
      writeSpeed(arg1, arg2);
      break;
    }

    case READ_SPEED: {
      //read actual speed
      float v0, v1;
      readSpeed(v0, v1);
      send_data(v0, v1);
      break;
    }

    case WRITE_PWM: {
      // write PWM
      writePWM((int)arg1, (int)arg2);
      break;
    }

    case READ_POS: {
      float pos0, pos1;
      // read pos
      readPos(pos0, pos1);
      send_data(pos0, pos1);
      break;
    }

    case READ_TSPEED: {
      float v0, v1;
      // read target speed
      readTargetSpeed(v0, v1);
      send_data(v0, v1);
      break;
    }

    case SET_PPR: {
      // write PPR
      setEncoderPPR((int)arg1, (double)arg2);
      break;
    }

    case GET_PPR: {
      //read PPR
      float res = getEncoderPPR((int)arg1);
      send_data(res);
      break;
    }

    case SET_KP: {
      //write KP
      setMotorKp((int)arg1, (double)arg2);
      break;
    }

    case GET_KP: {
      //read KP
      float res = getMotorKp((int)arg1);
      send_data(res);
      break;
    }


    case SET_KI: {
      //write KI
      setMotorKi((int)arg1, (double)arg2);
      break;
    }

    case GET_KI: {
      //read KI
      float res = getMotorKi((int)arg1);
      send_data(res);
      break;
    }

    
    case SET_KD: {
      //write KD
      setMotorKd((int)arg1, (double)arg2);
      break;
    }

    case GET_KD: {
      //read KD
      float res = getMotorKd((int)arg1);
      send_data(res);
      break;
    }


    case SET_RDIR: {
      //write RDIR
      setRdir((int)arg1, (double)arg2);
      break;
    }

    case GET_RDIR: {
      //read KI
      float res = getRdir((int)arg1);
      send_data(res);
      break;
    }

    case SET_CF: {
      //write Cutoff Freq
      setCutoffFreq((int)arg1, (double)arg2);
      break;
    }

    case GET_CF: {
      //read Cutoff Freq
      float res = getCutoffFreq((int)arg1);
      send_data(res);
      break;
    }

    case SET_MAX_SPEED: {
      //write Motor Max Speed
      setMaxVel((int)arg1, (double)arg2);
      break;
    }

    case GET_MAX_SPEED: {
      //read Motor Max Speed
      float res = getMaxVel((int)arg1);
      send_data(res);
      break;
    }

    case SET_PID_MODE: {
      //write PID Mode
      setPidModeFunc((int)arg2);
      break;
    }

    case GET_PID_MODE: {
      //read PID Mode
      float res = getPidModeFunc();
      send_data(res);
      break;
    }

    case SET_CMD_TIMEOUT: {
      //write Command Timeout
      setCmdTimeout((int)arg1);
      break;
    }

    case GET_CMD_TIMEOUT: {
      //read Command Timeout
      float res = getCmdTimeout();
      send_data(res);
      break;
    }

    case SET_I2C_ADDR: {
      //write I2C Address
      setI2cAddress((int)arg1);
      break;
    }

    case GET_I2C_ADDR: {
      //read I2C Address
      float res = getI2cAddress();
      send_data(res);
      break;
    }


    case RESET: {
      //reset all stored parameters return 1.0 if successfull
      float res = triggerResetParams();
      send_data(res);
      break;
    }

    case CLEAR: {
      // clear all inintializing variables
      float res = clearDataBuffer();
      send_data(res);
      break;
    }
  }

  gpio_set_level((gpio_num_t)LED_PIN, 0);
}


void recieve_and_send_data() {
  static char rx_buffer[64];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = Serial.read();

    // End of line → parse
    if (c == '\n' || c == '\r') {
      rx_buffer[idx] = null_char;
      idx = 0;

      char *ptr = rx_buffer;
      char *end;

      // Parse cmd
      float cmd_  = strtof(ptr, &end);
      cmd = (int)cmd_;
      ptr = end;

      // Parse arg1
      arg1 = strtof(ptr, &end);
      ptr = end;

      // Parse arg2
      arg2 = strtof(ptr, &end);

      runCommand();
      return;
    }

    // Store characters safely
    if (idx < sizeof(rx_buffer) - 1) {
      rx_buffer[idx++] = c;
    }
  }
}


#endif