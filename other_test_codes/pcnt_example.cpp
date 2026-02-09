#include <Arduino.h>
#include "driver/pcnt.h"

// motor 0 H-Bridge Connection
int IN1_0 = 26, IN2_0 = 27, EN_0 = 12;
// motor 1 H-Bridge Connection
int IN1_1 = 18, IN2_1 = 19, EN_1 = 23;
// motor 2 H-Bridge Connection
int IN1_2 = 33, IN2_2 = 25, EN_2 = 32;
// motor 3 H-Bridge Connection
int IN1_3 = 17, IN2_3 = 5, EN_3 = 16;

// motor 0 encoder connection
int enc0_A = 36, enc0_B = 39;
// motor 1 encoder connection
int enc1_A = 34, enc1_B = 35;
// motor 2 encoder connection
int enc2_A = 13, enc2_B = 14;
// motor 3 encoder connection
int enc3_A = 15, enc3_B = 4;

static const int32_t MAX_INT_RANGE = 32767;
static const int32_t MIN_INT_RANGE = -32768;
static const int32_t MAX_DELTA_COUNT_RANGE = 25000; // check PCNT Overflow

struct EncoderPCNT {
  pcnt_unit_t unit;
  int pulse_pin;
  int ctrl_pin;
};

struct EncoderState {
  int32_t prevCount;
  int32_t totalCount;
};

// Your encoder pins
EncoderPCNT encoders[4] = {
  {PCNT_UNIT_0, enc0_A, enc0_B},
  {PCNT_UNIT_1, enc1_A, enc1_B},
  {PCNT_UNIT_2, enc2_A, enc2_B},
  {PCNT_UNIT_3, enc3_A, enc3_B},
};

EncoderState encoderState[4];

void setup_pcnt(const EncoderPCNT &e)
{
  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = e.pulse_pin;
  cfg.ctrl_gpio_num  = e.ctrl_pin;
  cfg.channel        = PCNT_CHANNEL_0;
  cfg.unit           = e.unit;

  // Count on rising edge only (minimal)
  cfg.pos_mode = PCNT_COUNT_INC;
  cfg.neg_mode = PCNT_COUNT_DIS;

  // Direction control from ctrl pin
  cfg.lctrl_mode = PCNT_MODE_REVERSE;
  cfg.hctrl_mode = PCNT_MODE_KEEP;

  cfg.counter_h_lim = MAX_INT_RANGE;
  cfg.counter_l_lim = MIN_INT_RANGE;

  pcnt_unit_config(&cfg);

  // Optional glitch filter (very recommended)
  pcnt_set_filter_value(e.unit, 100); // ~1.25us
  pcnt_filter_enable(e.unit);

  pcnt_counter_pause(e.unit);
  pcnt_counter_clear(e.unit);
  pcnt_counter_resume(e.unit);
}

void update_encoder(EncoderState &e, pcnt_unit_t unit)
{
  int16_t currentCount16bit;
  pcnt_get_counter_value(unit, &currentCount16bit);

  int32_t currentCount  = (int32_t)currentCount16bit;

  int32_t deltaCount = currentCount - e.prevCount;

  if (abs(deltaCount) > MAX_DELTA_COUNT_RANGE) {
    if (deltaCount < 0){
      deltaCount =  MAX_INT_RANGE + deltaCount;
    }
    else if (deltaCount > 0){
      deltaCount =  MIN_INT_RANGE + deltaCount;
    }
  }

  e.totalCount += deltaCount;
  e.prevCount = currentCount;

  Serial.print("delta: ");
  Serial.print(deltaCount);
  Serial.print("   ");

}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(encoders[i].pulse_pin, INPUT_PULLUP);
    pinMode(encoders[i].ctrl_pin, INPUT_PULLUP);
    setup_pcnt(encoders[i]);
  }

  Serial.println("PCNT encoder demo started");
}

void loop() {
  for (int i = 0; i < 4; i++) {
    update_encoder(encoderState[i], encoders[i].unit);
    Serial.print("M");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(encoderState[i].totalCount);
    Serial.print("   ");
  }
  Serial.println();

  delay(150);
}