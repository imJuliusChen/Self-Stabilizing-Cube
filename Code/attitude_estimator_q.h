// Declares the attitude estimator.

#ifndef ATTITUDE_ESTIMATOR_Q_H
#define ATTITUDE_ESTIMATOR_Q_H

#include "parameters.h"
#include "math_utils.h"
#include "mpu6050.h"

struct Q_t {
    // volatile float q0;
    // volatile float q1;
    // volatile float q2;
    // volatile float q3;
    float q0;
    float q1;
    float q2;
    float q3;
};

struct EulerAngle_t {
    float roll;  // Roll angle (rad)
    float pitch;  // Pitch angle (rad)
    float yaw;  // Yaw angle (rad)
};


class AttitudeEstimator {
public:
  AttitudeEstimator();
  void init();
  void updateDeviceReadings(float Gyro_sensor[3], float Acce_sensor[3]);  // Update internal Gyro_device and Acce_device
  void estimateAngularVelocity(float GyroRateEuler[3]);
  void estimate(float elapsedTime,
                float& AngleRoll_Kal, float& AnglePitch_Kal, float& KalUncerAngleRoll, float& KalUncerAnglePitch,
                float GyroRateEuler[3]);
  Q_t q_;  // Quaternion representing orientation
  Q_t q_2;
  EulerAngle_t EuAngle_;
  void ImuAHRSUpdate(float elapsedTime, Q_t &_q);  // Update quaternion using AHRS (Attitude and Heading Reference System) algorithm
  void ConvertToEulerAngleByQ(EulerAngle_t &_EuAngle, const Q_t &_q);  // Convert quaternion to Euler angles
  // float corrected_gyro[3];  // Corrected gyro rates (rad/s)
  void estimateAngularVelocityFromQ(float GyroRateEuler[3], const Q_t& _q);
  void complementaryQ(float dt, Q_t &_q);
  void getAcceDevice(float acce_data[3]) const;  // Getter to safely read the private Acce_device data

private:
  float DCM_SensorToDevice[3][3];  // fixed DCM computed from fixed Euler angles
  float Gyro_device[3];  // transform expression of vector's components from sensor frame to device frame  // sequence: X, Y, Z
  float Acce_device[3];  // transform expression of vector's components from sensor frame to device frame  // sequence: X, Y, Z

  float AngleRoll_Acce_v1, AnglePitch_Acce_v1;
  // float AngleRoll_Acce_v2, AnglePitch_Acce_v2;
  // float AngleRoll_Acce_v3, AnglePitch_Acce_v3;
  // float AngleRoll_Acce_v4, AnglePitch_Acce_v4;

  float Kalman1DOutput[2];
  // {Kalman prediction of the state (angle), uncertainty of the prediction}
  // both variables are updated during each iteration

  void kalman_1d(float& KalState, float& KalUncer, float KalInput, float KalMeas, float KalElapsedT);

  float kp_;  // Proportional gain for AHRS algorithm, controls convergence rate to accelerometer data
  float ki_;  // Integral gain for AHRS algorithm, controls convergence rate of gyro bias correction
};

#endif