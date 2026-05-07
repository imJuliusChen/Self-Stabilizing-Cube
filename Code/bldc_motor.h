// Declares the SimpleFOC motor interface.

#ifndef BLDC_MOTOR_H
#define BLDC_MOTOR_H

#include <SimpleFOC.h>
#include "pin_names.h"
#include "magnetic_sensor.h"

class BLDCMotorController {
public:
  BLDCMotorController(int pwmA, int pwmB, int pwmC, int enable);
  void init();
  void linkSensor(MagneticSensor* sensor);
  void loopFOC();
  void move(float target_voltage);
  float getShaftVelocity();
  float getMaxVoltage() const { return driver.voltage_power_supply * 0.7; }  // Add a public getter method

private:
  BLDCMotor motor;
  BLDCDriver3PWM driver;

  // BLDCMotor(pole pair number, phase resistance(optional));
  // BLDCMotor motor = BLDCMotor(7);

  // BLDCDriver3PWM(pwmA, pwmB, pwmC, enable(optional));
  /*
    To create the interface to the BLDC driver you need to specify the 3 A,B,C phase pwm pin numbers for each motor phase and optionally input enable pin
    BLDCDriver3PWM(int phA, int phB, int phC, int en)
    try to use the PWM pins that belong to the same timer. For 3PWM mode, try using only one timer.
    avoid using pins from Timer0 (pin 5, 6)
  */
  // BLDCDriver3PWM driver = BLDCDriver3PWM(26, 25, 33, 32);  
};

#endif