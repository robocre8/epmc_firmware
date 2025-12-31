#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include "command_functions.h"

// The arguments converted to integers
int cmd;
int cmd_pos;
float arg1;
float arg2;

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

  float r0 = 0.0f;
  float r1 = 0.0f;

  switch (cmd) {
    case WRITE_SPEED: {
      writeSpeed(arg1, arg2);
      break;
    }

    case READ_SPEED: {
      //read actual speed
      readSpeed(r0, r1);
      send_data(r0, r1);
      break;
    }

    case WRITE_PWM: {
      // write PWM
      writePWM((int)arg1, (int)arg2);
      break;
    }

    case READ_POS: {
      // read pos
      readPos(r0, r1);
      send_data(r0, r1);
      break;
    }

    case READ_TSPEED: {
      // read target speed
      readTargetSpeed(r0, r1);
      send_data(r0, r1);
      break;
    }

    case SET_PPR: {
      // write PPR
      setEncoderPPR((int)arg1, (double)arg2);
      break;
    }

    case GET_PPR: {
      //read PPR
      r0 = getEncoderPPR((int)arg1);
      send_data(r0, r1);
      break;
    }

    case SET_KP: {
      //write KP
      setMotorKp((int)arg1, (double)arg2);
      break;
    }

    case GET_KP: {
      //read KP
      r0 = getMotorKp((int)arg1);
      send_data(r0, r1);
      break;
    }


    case SET_KI: {
      //write KI
      setMotorKi((int)arg1, (double)arg2);
      break;
    }

    case GET_KI: {
      //read KI
      r0 = getMotorKi((int)arg1);
      send_data(r0, r1);
      break;
    }

    
    case SET_KD: {
      //write KD
      setMotorKd((int)arg1, (double)arg2);
      break;
    }

    case GET_KD: {
      //read KD
      r0 = getMotorKd((int)arg1);
      send_data(r0, r1);
      break;
    }

    case SET_RDIR: {
      //write RDIR
      setRdir((int)arg1, (double)arg2);
      break;
    }

    case GET_RDIR: {
      //read KI
      r0 = getRdir((int)arg1);
      send_data(r0, r1);
      break;
    }

    case SET_CF: {
      //write Cutoff Freq
      setCutoffFreq((int)arg1, (double)arg2);
      break;
    }

    case GET_CF: {
      //read Cutoff Freq
      r0 = getCutoffFreq((int)arg1);
      send_data(r0, r1);
      break;
    }

    case SET_MAX_SPEED: {
      //write Motor Max Speed
      setMaxSpeed((int)arg1, (double)arg2);
      break;
    }

    case GET_MAX_SPEED: {
      //read Motor Max Speed
      r0 = getMaxSpeed((int)arg1);
      send_data(r0, r1);
      break;
    }

    case SET_PID_MODE: {
      //write PID Mode
      setPidModeFunc((int)arg2);
      break;
    }

    case GET_PID_MODE: {
      //read PID Mode
      r0 = getPidModeFunc();
      send_data(r0, r1);
      break;
    }

    case SET_CMD_TIMEOUT: {
      //write Command Timeout
      setCmdTimeout((int)arg2);
      break;
    }

    case GET_CMD_TIMEOUT: {
      //read Command Timeout
      r0 = getCmdTimeout();
      send_data(r0, r1);
      break;
    }

    case SET_I2C_ADDR: {
      //write I2C Address
      setI2cAddress((int)arg2);
      break;
    }

    case GET_I2C_ADDR: {
      //read I2C Address
      r0 = getI2cAddress();
      send_data(r0, r1);
      break;
    }


    case RESET: {
      //reset all stored parameters return 1.0 if successfull
      r0 = triggerResetParams();
      send_data(r0, r1);
      break;
    }

    case CLEAR: {
      // clear all inintializing variables
      r0 = clearDataBuffer();
      send_data(r0, r1);
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