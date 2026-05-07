// Defines hardware pins assignments.

#ifndef PIN_NAMES_H
#define PIN_NAMES_H

// Buzzer pin definition
#define BUZZER_PIN 23  // Connect buzzer to GPIO 23

// I2C pins for MPU-6050
// Use specific I2C pins for ESP32 (default SDA: 21, SCL: 22), ESP32 allows I2C pin remapping
#define SDA_PIN 21
#define SCL_PIN 22

// BLDC motor PWM and enable pins
// Motor1: ESP32 PWM pins (GPIO 25, 26, 27), enable on GPIO 32
#define MOTOR1_PWM_A 25
#define MOTOR1_PWM_B 26
#define MOTOR1_PWM_C 27
#define MOTOR1_ENABLE 32
// Motor2: ESP32 PWM pins (GPIO 16, 17, 18), enable on GPIO 19
#define MOTOR2_PWM_A 16
#define MOTOR2_PWM_B 17
#define MOTOR2_PWM_C 18
#define MOTOR2_ENABLE 19
// Motor3: ESP32 PWM pins (GPIO 13, 14, 15), enable on GPIO 33
#define MOTOR3_PWM_A 13
#define MOTOR3_PWM_B 14
#define MOTOR3_PWM_C 15
#define MOTOR3_ENABLE 33

#endif