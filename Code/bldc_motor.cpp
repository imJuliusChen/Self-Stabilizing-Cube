// Implements motor initialization and control.

#include "bldc_motor.h"

// BLDCMotorController::BLDCMotorController() : motor(7), driver(MOTOR_PWM_A, MOTOR_PWM_B, MOTOR_PWM_C, MOTOR_ENABLE) {}  // Ensures the object is initialized before any code in the constructor body runs
BLDCMotorController::BLDCMotorController(int pwmA, int pwmB, int pwmC, int enable) : motor(7), driver(pwmA, pwmB, pwmC, enable) {}


void BLDCMotorController::init() {
  // Driver configuration
  // pwm frequency to be used [Hz]: for atmega328 fixed to either 4k or 32kHz; esp32/stm32/teensy configurable
  // driver.pwm_frequency = 32000;

  // Power supply voltage [V]
  driver.voltage_power_supply = 12;

  // Limit the maximal dc voltage the driver can set as a protection measure for the low-resistance motors, this value is fixed on startup, default: voltage_power_supply
  // driver.voltage_limit = 12;

  // Initialize driver
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    while (1);  // Halt execution
  }
  Serial.println("Driver initialized");
  // Link the motor to the driver
  motor.linkDriver(&driver);

  // Aligning voltage 
  motor.voltage_sensor_align = 5;

  // Limiting motor movements
  // limit the voltage to be set to the motor, start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  // motor.voltage_limit = 3;   // [V]

  /*
  Hierarchy: The effective voltage applied to the motor is the minimum of:
  - target_voltage (from code)
  - motor.voltage_limit (software)
  - driver.voltage_limit (hardware)
  - driver.voltage_power_supply (physical supply limit)
  */
  
  // choose FOC modulation (optional)
  // motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // Motor configuration
  // motor.torque_controller = TorqueControlType::voltage;  // default  // Set torque mode
  motor.controller = MotionControlType::torque;  // Set motion control loop type to be used
  // motor.controller = MotionControlType::velocity;  // Set motion control loop type to be used

  // Initialize motor
  if (!motor.init()) {
    Serial.println("Motor init failed!");
    while (1);  // Halt execution
  }
  Serial.println("Motor initialized");

  // Align sensor and start FOC
  motor.initFOC();
  Serial.println("FOC initialized");
}


void BLDCMotorController::linkSensor(MagneticSensor* sensor) {
  // Access MagneticSensorI2C object
  motor.linkSensor(&sensor->getSensor());
}


void BLDCMotorController::loopFOC() {
  motor.loopFOC();
}


void BLDCMotorController::move(float target_voltage) {
  // Motion control function (Set the target voltage to the motor)
  // this function can be run at much lower frequency than loopFOC() function
  motor.move(target_voltage);
  // motor.move(30);
}


float BLDCMotorController::getShaftVelocity() {
  return motor.shaft_velocity;
}