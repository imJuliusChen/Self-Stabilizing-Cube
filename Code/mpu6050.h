// Declares the MPU-6050 driver interface.

#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include <SimpleFOC.h>
#include "pin_names.h"

class MPU6050 {
public:
  MPU6050(TwoWire* w);
  void init();

  void calibrate(float& GyroCaliX, float& GyroCaliY, float& GyroCaliZ);
  
  void readSensors(float* Gyro_sensor, float* Acce_sensor, float GyroCaliX, float GyroCaliY, float GyroCaliZ);

private:
  // float GyroCaliX, GyroCaliY, GyroCaliZ;
  TwoWire* wire;
};

#endif