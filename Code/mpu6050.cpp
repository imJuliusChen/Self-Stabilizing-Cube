// Implements MPU-6050 initialization and reading.

#include "mpu6050.h"

// MPU6050::MPU6050() : GyroCaliX(0), GyroCaliY(0), GyroCaliZ(0) {}
MPU6050::MPU6050(TwoWire* w) {
  wire = w;
}

void MPU6050::init() {

  // Use specific I2C pins for ESP32 (default SDA: 21, SCL: 22), ESP32 allows I2C pin remapping
  // Wire.begin(SDA_PIN, SCL_PIN);  // Initialize I2C communication
  // Wire.setClock(400000);  // set the I2C transmission rate (clock speed of I2C) (HZ)
  _delay(250);  // to give MPU-6050 time to start

  // MPU-6050 initialization  // communication with the gyroscope and calibration
  wire->beginTransmission(0x68);  // start transmitting the address
  wire->write(0x6B);  // Wake up MPU-6050  // start the gyro in power mode, write to the power management register
  wire->write(0x00);
  wire->endTransmission();  // end transmission and release the bus, return a number representing the transmission status, 0 indicating success  // terminate the connection with the gyroscope

  
  // Configure low-pass filter
  // Filter's characteristics are set by the DLPF_CFG bits, the last three bits (bits 0-2) of the CONFIG register.
  // A higher bandwidth allows for faster response to changes in motion, while a lower bandwidth provides more smoothing and noise reduction.
  wire->beginTransmission(0x68);  // start I2C communication with the gyro, each device has its unique address with I2C protocol
  wire->write(0x1A);  // low-pass filter register
  // wire->write(0x05);  // enable filter, configures the filter to a bandwidth of 10 Hz for both the accelerometer and the gyroscope
  // wire->write(0x04);  // enable filter, bandwidth 20 Hz, faster response
  wire->write(0x03);  // enable filter, bandwidth 42-44 Hz, faster response, vibrations shaking faster than ~44 Hz are filtered out.
  // wire->write(0x02);
  wire->endTransmission();
  

  // Configure gyroscope sensitivity (±500°/s)
  wire->beginTransmission(0x68);
  wire->write(0x1B);  // configure the gyro output and pull rotation rate measurements from the sensor, set the sensitivity scale factor
  wire->write(0x08);  // ±500°/s
  wire->endTransmission();

  // Configure accelerometer range (±8g)
  wire->beginTransmission(0x68);
  wire->write(0x1C);  // configure the acce output, acce configuration settings are stored in register 1C
  wire->write(0x10);  // choose a full-scale range of 8G
  wire->endTransmission();

  _delay(100);  // Add delay to allow I2C bus to settle
}


void MPU6050::calibrate(float& GyroCaliX, float& GyroCaliY, float& GyroCaliZ) {
  
  // hold it stationary during the first 1 second after uploaded the code
  for (int i = 0; i < 1000; i++) {  // perform the calibration measurements
    float Gyro_sensor[3], Acce_sensor[3];
    readSensors(Gyro_sensor, Acce_sensor, 0, 0, 0);  // No calibration during calibration
    GyroCaliX += Gyro_sensor[0];
    GyroCaliY += Gyro_sensor[1];
    GyroCaliZ += Gyro_sensor[2];
    _delay(1);  // delay 1 milli-second after the other, so this step will take 1 second in total
  }
  // storing the results in the object’s private member variables
  GyroCaliX /= 1000;  // calculate the calibration values
  GyroCaliY /= 1000;
  GyroCaliZ /= 1000;
}


// read the rotation rates and acceleration from the MPU-6050
// Stores/outputs the results in Gyro_sensor and Acce_sensor
void MPU6050::readSensors(float* Gyro_sensor, float* Acce_sensor, float GyroCaliX, float GyroCaliY, float GyroCaliZ) {
  
  // Burst read gyro and accel data (14 bytes: 6 accel, 2 temp, 6 gyro)
  wire->beginTransmission(0x68);
  wire->write(0x3B);  // access registers storing acce and gyro measurements, indicate the first register we will use
  wire->endTransmission();

  wire->requestFrom(0x68,14);  // request 14 bytes from the sensor, pull information of the 14 registers 3B to 40 & 43 to 48
  int16_t AcceXLSB = wire->read() << 8 | wire->read();  // read the acce measurements around the X axis, declare as an unsigned 16-bit integer, measurement is spread out over 2 registers with each 8-bits, merge by calling wire.read twice
  int16_t AcceYLSB = wire->read() << 8 | wire->read();
  int16_t AcceZLSB = wire->read() << 8 | wire->read();
  wire->read(); wire->read();  // Skip temperature data (2 bytes)
  int16_t GyroXLSB = wire->read() << 8 | wire->read();  // read the gyro measurements around the X axis, declare as an unsigned 16-bit integer, measurement is spread out over 2 registers with each 8-bits, merge by calling wire.read twice
  int16_t GyroYLSB = wire->read() << 8 | wire->read();
  int16_t GyroZLSB = wire->read() << 8 | wire->read();


  // convert the measurement units from LSB to °/sec, measurements are expressed in LSB
  // shift coordinate definition of MPU6050 on purpose !!!
  // shift Y to X, X to Y, -Z to Z
  Gyro_sensor[0] = (float)GyroYLSB / 65.5f - GyroCaliX;  // correct the measured values
  Gyro_sensor[1] = (float)GyroXLSB / 65.5f - GyroCaliY;
  Gyro_sensor[2] = (float)-GyroZLSB / 65.5f - GyroCaliZ;

  // convert the measurement units from LSB to g (physical values)
  // shift coordinate definition of MPU6050 on purpose !!!
  // shift Y to X, X to Y, -Z to Z
  Acce_sensor[0] = (float)AcceYLSB / 4096.0f + 0.0f;
  Acce_sensor[1] = (float)AcceXLSB / 4096.0f + 0.0f;
  Acce_sensor[2] = (float)-AcceZLSB / 4096.0f + 0.0f;
}