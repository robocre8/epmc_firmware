#include <Arduino.h>
#include "encoder_setup_pcnt.h"

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

double enc_ppr[4]={
  1092.0, // motor 0 encoder pulse per revolution parameter
  1092.0, // motor 1 encoder pulse per revolution parameter
  1092.0, // motor 2 encoder pulse per revolution parameter
  1092.0 // motor 3 encoder pulse per revolution parameter
};

QuadEncoderPCNT encoder[4] = {
  QuadEncoderPCNT(enc0_A, enc0_B, enc_ppr[0], PCNT_UNIT_0), // motor 0 encoder connection
  QuadEncoderPCNT(enc1_A, enc1_B, enc_ppr[1], PCNT_UNIT_1), // motor 1 encoder connection
  QuadEncoderPCNT(enc2_A, enc2_B, enc_ppr[2], PCNT_UNIT_2), // motor 2 encoder connection
  QuadEncoderPCNT(enc3_A, enc3_B, enc_ppr[3], PCNT_UNIT_3) // motor 3 encoder connection
};

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    encoder[i].begin();
  }

  Serial.println("PCNT encoder demo started");
}

void loop() {
  for (int i = 0; i < 4; i++) {
    encoder[i].update_total_encoder_count();
    Serial.print("M");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(encoder[i].getAngVel());
    Serial.print("   ");
  }
  Serial.println();

  delay(150);
}