#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "AbsoluteEncoder.h"

#include "TractionEncoder.h"
#include "MovingAvgFilter.h"
#include "ExpSmoothingFilter.h"

#include "SmartMotor.h"
#include "Motor.h"
#include "PID_v1.h"
#include "PID_AutoTune_v0.h"

#include "include/definitions.h"
#include "include/mod_config.h"
#include "include/communication.h"

int time_bat = 0;
int time_tel = 0;
int time_data = 0;
int time_tel_avg = DT_TEL;

SmartMotor motorTrLeft(DRV_TR_LEFT_PWM, DRV_TR_LEFT_DIR, ENC_TR_LEFT_A, ENC_TR_LEFT_B, false);
SmartMotor motorTrRight(DRV_TR_RIGHT_PWM, DRV_TR_RIGHT_DIR, ENC_TR_RIGHT_A, ENC_TR_RIGHT_B, true);

// === PID ===
double setpoint = 100.0;
double input = 0;
double output = 0;

double Kp = 1.0, Ki = 0.5, Kd = 0.1;
PID_v1 myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// === Autotune ===
PID_ATune tuner(&input, &output);
bool tuning = true;

void setup() {
  Serial.begin(115200);

  tuner.SetOutputStep(50);
  tuner.SetControlType(1);  // PID
  tuner.SetLookbackSec(10);
  tuner.SetNoiseBand(1);

  Serial.println("== Inizio AutoTune PID ==");
 while (tuning) {
    input = motorTrLeft.getSpeed();

    if (tuner.Runtime()) {
      output = constrain(output, -PWM_MAX_VALUE, PWM_MAX_VALUE);
      motorTrLeft.write_d(output);
    } else {
      Kp = tuner.GetKp();
      Ki = tuner.GetKi();
      Kd = tuner.GetKd();
      myPID.SetTunings(Kp, Ki, Kd);
      myPID.SetMode(AUTOMATIC);
      tuning = false;

      Serial.println("== AutoTune completato ==");
      Serial.print("Kp: "); Serial.println(Kp);
      Serial.print("Ki: "); Serial.println(Ki);
      Serial.print("Kd: "); Serial.println(Kd);
    }

    delay(100);
  }
}

void loop() {
  delay(1000);  // Piccolo delay per evitare di saturare la CPU
}
