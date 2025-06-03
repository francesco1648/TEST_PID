#include "Motor.h"

/**
 * Create object and set motor pins.
 * @param pwm PWM pin.
 * @param dir Direction pin.
 * @param invert Invert motor direction, usuful when motors are mounted opposite to one another.
 */
Motor::Motor(byte pwm, byte dir, bool invert)
    : pwm(pwm), dir(dir), invert(invert)
{}

/**
 * Initialize motors.
 */
void Motor::begin() {
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
}

/**
 * Sets the motor speed.
 * @param value Speed of the motor, ranging from 0 to maximum PWM value.
 */
void Motor::write(int value) {
  int sign = (value < 0) ? -1 : (value > 0 ? 1 : 0);
  int mot = constrain(abs(value), 0, PWM_MAX_VALUE);

  const int MIN_PWM = 50; // sotto questo valore il motore non si muove

  // Se c'è segnale e siamo sotto la soglia, alza al minimo efficace
  if (mot > 0 && mot < MIN_PWM){
mot = 0;
  }


  analogWrite(pwm, mot);
  digitalWrite(dir, invert ^ (sign < 0));

  Serial.print("\tanalogWrite\t");
  Serial.print(mot);
  Serial.print("\tdigitalWrite\t");
  Serial.print(invert ^ (sign < 0));
}

