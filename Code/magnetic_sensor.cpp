// Implements magnetic sensor initialization and control.

#include "magnetic_sensor.h"

// MagneticSensor::MagneticSensor() : sensor(AS5600_I2C) {}  // Ensures the object is initialized before any code in the constructor body runs
// MagneticSensor::MagneticSensor(uint8_t channel) : sensor(AS5600_I2C_ADDRESS, 12, 0x0E, 4), mux_channel(channel) {}
MagneticSensor::MagneticSensor(uint8_t id, int bit_res, int angle_reg, int msb_bits_used, TwoWire* i2c_bus) : sensor(id, bit_res, angle_reg, msb_bits_used), wire(i2c_bus), i2c_address(id) {}  // Initialize with specific parameters


/*
void MagneticSensor::selectMuxChannel() {
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << mux_channel);  // Select the specified channel
  Wire.endTransmission();
}
*/


// Private helper function to write the filter profile to the AS5600's CONF register
void MagneticSensor::_setFilterProfile(uint8_t conf_high_byte) {
  // The CONF register is two bytes: 0x07 (low) and 0x08 (high).
  // The profile value we pass in sets the entire high byte (0x08).
  
  // selectMuxChannel();
  
  // We read the low byte (0x07) first to avoid changing unrelated settings
  wire->beginTransmission(i2c_address);
  wire->write(0x07);  // Point to the start of the CONF register
  wire->endTransmission();
  
  wire->requestFrom((int)i2c_address, 1);
  uint8_t conf_low = wire->read();
  
  // Write the preserved low byte and the new high byte (our profile) back to the sensor.
  wire->beginTransmission(i2c_address);
  wire->write(0x07);  // Point to the start of the CONF register
  wire->write(conf_low);
  wire->write(conf_high_byte);
  wire->endTransmission();
  
  _delay(2);  // Allow a moment for the setting to take effect.
}


void MagneticSensor::init() {
  // selectMuxChannel();
  // sensor.init();
  sensor.init(wire);  // Initialize on the specific bus (Wire or Wire1)
  // Serial.println("Magnetic sensor initialized");
  // Serial.print("Magnetic sensor initialized on mux channel ");
  // Serial.println(mux_channel);
  Serial.print("Magnetic Sensor initialized at 0x");
  Serial.println(i2c_address, HEX);

  
  // AS5600 FILTER PROFILE CONFIGURATION
  // These profiles combine the Slow Filter (for noise) and the Fast Filter (for responsiveness).
  
  // Profile Name:        Slow Filter:     Fast Filter Threshold:    Use Case:
  // PROFILE_RESPONSIVE   Fastest (2x)     6 LSB (Very Sensitive)    Tracking rapid, jerky movements.
  // PROFILE_BALANCED     Medium (8x)      10 LSB (Balanced)         General purpose, good mix of speed and smoothness.
  // PROFILE_SMOOTH       Slowest (16x)    18 LSB (Less Sensitive)   Prioritizing noise reduction and stability.
  // PROFILE_SLOW_ONLY    Medium (8x)      Disabled                  For very smooth, predictable motion.
  
  if (i2c_address == 0x36) {  // Only apply AS5600 filter if the address matches AS5600
    //_setFilterProfile(AS5600_FILTER_PROFILE_RESPONSIVE);
    _setFilterProfile(AS5600_FILTER_PROFILE_BALANCED);
    // _setFilterProfile(AS5600_FILTER_PROFILE_SMOOTH);
    //_setFilterProfile(AS5600_FILTER_PROFILE_SLOW_ONLY);
    Serial.println("AS5600 filter profile set.");
  } else {
    Serial.println("Default settings.");
  }
}


void MagneticSensor::update() {
  // selectMuxChannel();
  sensor.update();
}


float MagneticSensor::getAngle() {
  // selectMuxChannel();
  return sensor.getAngle();
}


float MagneticSensor::getVelocity() {
  // selectMuxChannel();
  return sensor.getVelocity();
}