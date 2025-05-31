
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


#include "include/definitions.h"
#include "include/mod_config.h"
#include "include/communication.h"




int time_bat = 0;
int time_tel = 0;
int time_data = 0;
int time_tel_avg = DT_TEL;



SmartMotor motorTrLeft(DRV_TR_LEFT_PWM, DRV_TR_LEFT_DIR, ENC_TR_LEFT_A, ENC_TR_LEFT_B, false);
SmartMotor motorTrRight(DRV_TR_RIGHT_PWM, DRV_TR_RIGHT_DIR, ENC_TR_RIGHT_A, ENC_TR_RIGHT_B, true);

/********************************************************
 * PID_v1 Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID_v1 myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  Input = analogRead(motorTrLeft.getSpeed());
  Setpoint = 100;

  //turn the PID_v1 on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input = analogRead(motorTrLeft.getSpeed());
  myPID.Compute();
  motorTrLeft.write_d(Output);
}