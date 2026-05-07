// Declares the SimpleFOC magnetic sensor interface.

#ifndef MAGNETIC_SENSOR_H
#define MAGNETIC_SENSOR_H

#include <SimpleFOC.h>
#include <Wire.h>

// #define TCA9548A_ADDRESS 0x70  // I2C address of TCA9548A multiplexer
#define AS5600_I2C_ADDRESS 0x36  // Fixed AS5600 address

// Combined Filter Profiles for AS5600 CONF Register (High Byte)
// These profiles configure both the Slow Filter (SF) for noise and Fast Filter Threshold (FTH) for responsiveness.
// Register 0x08 Bit Structure: [WD] [FTH2] [FTH1] [FTH0] [SF1] [SF0] [res] [res]
#define AS5600_FILTER_PROFILE_RESPONSIVE  0b00011100  // SF=2x (Fastest), FTH=6 LSB (Most Sensitive)
#define AS5600_FILTER_PROFILE_BALANCED    0b01110100  // SF=8x (Medium), FTH=10 LSB (Balanced)
#define AS5600_FILTER_PROFILE_SMOOTH      0b01000000  // SF=16x (Slowest), FTH=18 LSB (Least Sensitive)
#define AS5600_FILTER_PROFILE_SLOW_ONLY   0b00000100  // SF=8x (Medium), Fast Filter is Disabled

class MagneticSensor {
public:
  // MagneticSensor(uint8_t mux_channel);
  MagneticSensor(uint8_t id, int bit_res, int angle_reg, int msb_bits_used, TwoWire* i2c_bus);  // Constructor accepts specific sensor ID (address) and Bus
  void init();
  void update();
  // void selectMuxChannel();  // Select multiplexer channel
  
  MagneticSensorI2C& getSensor() { return sensor; }  // Add a getter
  
  float getAngle();
  float getVelocity();

private:
  MagneticSensorI2C sensor;

  // MagneticSensorI2C(uint8_t _chip_address, float _cpr, uint8_t _angle_register_msb)
  /*
  - chip_address:       I2C chip address of the magnetic sensor
  - bit_resolution:     resolution of the sensor (number of bits of the sensor internal counter register)
  - angle_register_msb: register number containing the msb part of the angle value (ex. AS5600 - 0x0E)
  - bits_used_msb:      number of used bits in msb register
    make sure to read the chip address and the chip angle register msb value from the datasheet
    also in most cases you will need external pull-ups on SDA and SCL lines!!!!!
  */
  // MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
  // MagneticSensorI2C sensor = MagneticSensorI2C(0x36, 12, 0x0E, 4);
  
  // uint8_t mux_channel;
  TwoWire* wire;  // Store the bus and address for this specific sensor
  uint8_t i2c_address;

  // Set the filter profile
  void _setFilterProfile(uint8_t conf_high_byte);
};

#endif