// Implements attitude estimation.

#include "attitude_estimator_q.h"

AttitudeEstimator::AttitudeEstimator() {
  memset(DCM_SensorToDevice, 0, sizeof(DCM_SensorToDevice));
  memset(Gyro_device, 0, sizeof(Gyro_device));  // Clear IMU data structure
  memset(Acce_device, 0, sizeof(Acce_device));  // Clear IMU data structure

  AngleRoll_Acce_v1 = 0;
  AnglePitch_Acce_v1 = 0;
  
  // Initialize output of the filter
  Kalman1DOutput[0] = 0;
  Kalman1DOutput[1] = 0;

  // Initialize quaternion (identity quaternion)
  q_.q0 = 1.0f;
  q_.q1 = 0;
  q_.q2 = 0;
  q_.q3 = 0;
  
  q_2.q0 = 1.0f;
  q_2.q1 = 0;
  q_2.q2 = 0;
  q_2.q3 = 0;

  // Initialize Euler angles
  // EuAngle_.roll = 0.0f;
  // EuAngle_.pitch = 0.0f;
  // EuAngle_.yaw = 0.0f;

  // AHRS gains - tuned for good performance
  // kp_ = 1.0f;
  // kp_ = 2.0f;  // Increased for faster convergence
  kp_ = 1.0f;
  // ki_ = 0.01f;
  ki_ = 0.005f;
}


// Initialize estimator
void AttitudeEstimator::init() {
  // float cpsi_val = pgm_read_float(&cpsi);
  // float spsi_val = pgm_read_float(&spsi);
  // float ctheta_val = pgm_read_float(&ctheta);
  // float stheta_val = pgm_read_float(&stheta);
  // float cphi_val = pgm_read_float(&cphi);
  // float sphi_val = pgm_read_float(&sphi);
  // computeDCM(DCM_SensorToDevice, cpsi_val, spsi_val, ctheta_val, stheta_val, cphi_val, sphi_val);

  // Directly assign the values of the DCM
  DCM_SensorToDevice[0][0] = -0.80903f;
  DCM_SensorToDevice[0][1] =  0.01800f;
  DCM_SensorToDevice[0][2] =  0.58749f;

  DCM_SensorToDevice[1][0] =  0.38520f;
  DCM_SensorToDevice[1][1] =  0.77121f;
  DCM_SensorToDevice[1][2] =  0.50681f;

  DCM_SensorToDevice[2][0] = -0.44396f;
  DCM_SensorToDevice[2][1] =  0.63633f;
  DCM_SensorToDevice[2][2] = -0.63087f;
}


// Update internal Gyro_device and Acce_device
void AttitudeEstimator::updateDeviceReadings(float Gyro_sensor[3], float Acce_sensor[3]) {
  applyTransM(Gyro_sensor, Gyro_device, DCM_SensorToDevice);  // apply DCM transformation, maps vectors from sensor frame to device frame
  // for (int i = 0; i < 3; i++) {
  //   Gyro_device[i] = Gyro_sensor[i]; // manually copy each element
  // }

  applyTransM(Acce_sensor, Acce_device, DCM_SensorToDevice);  // apply DCM transformation, maps vectors from sensor frame to device frame
  // for (int i = 0; i < 3; i++) {
  //   Acce_device[i] = Acce_sensor[i]; // manually copy each element
  // }
}


void AttitudeEstimator::estimateAngularVelocity(float GyroRateEuler[3]) {
  // compute the kinematic mapping matrix: from body frame to Euler angles
  // float KMM_BodyToEuler[3][3];
  // computeKMM(KMM_BodyToEuler, AnglePitch_Acce_v1, AngleRoll_Acce_v1);
  // applyTransM(Gyro_device, GyroRateEuler, KMM_BodyToEuler);  // apply transformation, maps vectors from body frame to Euler angles

  GyroRateEuler[0] = Gyro_device[2] * 0.01745f;  // Yaw  // *PI/180
  GyroRateEuler[1] = Gyro_device[1] * 0.01745f;  // Pitch
  GyroRateEuler[2] = Gyro_device[0] * 0.01745f;  // Roll
}


void AttitudeEstimator::estimate(float elapsedTime,
                                 float& AngleRoll_Kal, float& AnglePitch_Kal, float& KalUncerAngleRoll, float& KalUncerAnglePitch,
                                 float GyroRateEuler[3]) {
  // method1  // Carbon Aeronautics
  AngleRoll_Acce_v1 = atanf(Acce_device[1] / sqrtf(Acce_device[0] * Acce_device[0] + Acce_device[2] * Acce_device[2]));
  AnglePitch_Acce_v1 = -atanf(Acce_device[0] / sqrtf(Acce_device[1] * Acce_device[1] + Acce_device[2] * Acce_device[2]));
  // Small-angle approximation
  // AngleRoll_Acce_v1 = (Acce_device[1] / sqrtf(Acce_device[0] * Acce_device[0] + Acce_device[2] * Acce_device[2]));
  // AnglePitch_Acce_v1 = -(Acce_device[0] / sqrtf(Acce_device[1] * Acce_device[1] + Acce_device[2] * Acce_device[2]));
/*
  // method2  // Freescale Semiconductor
  AngleRoll_Acce_v2 = atan2f(Acce_device[1], Acce_device[2]);  // only this one is different from the others, the rest are the same
  AnglePitch_Acce_v2 = atanf(-Acce_device[0] / sqrtf(Acce_device[1] * Acce_device[1] + Acce_device[2] * Acce_device[2]));

  // method3  // DroneBot Workshop
  AngleRoll_Acce_v3 = asinf(Acce_device[1] / sqrtf(Acce_device[0] * Acce_device[0] + Acce_device[1] * Acce_device[1] + Acce_device[2] * Acce_device[2]));
  AnglePitch_Acce_v3 = asinf(-Acce_device[0] / sqrtf(Acce_device[0] * Acce_device[0] + Acce_device[1] * Acce_device[1] + Acce_device[2] * Acce_device[2]));

  // method4  // Orbital Mechanics for Engineering Students
  AngleRoll_Acce_v4 = atanf(Acce_device[1] / Acce_device[2]);
  AnglePitch_Acce_v4 = asinf(-Acce_device[0]);
*/


  // start the iteration for the Kalman filter with the roll and pitch angles
  kalman_1d(AngleRoll_Kal, KalUncerAngleRoll, GyroRateEuler[2], AngleRoll_Acce_v1, elapsedTime);
  AngleRoll_Kal = Kalman1DOutput[0];
  KalUncerAngleRoll = Kalman1DOutput[1];

  kalman_1d(AnglePitch_Kal, KalUncerAnglePitch, GyroRateEuler[1], AnglePitch_Acce_v1, elapsedTime);
  AnglePitch_Kal = Kalman1DOutput[0];
  KalUncerAnglePitch = Kalman1DOutput[1];
}


// KalState = angle calculated with the Kalman filter
// KalMeas = accelerometer measured angle
void AttitudeEstimator::kalman_1d(float& KalState, float& KalUncer, float KalInput, float KalMeas, float KalElapsedT) {
  // predicts the next state based on the previous state and gyroscope measurement (not the final prediction in this iteration)
  KalState += KalElapsedT * KalInput;
  // rotation angle from gyro = iteration length * rotation rate (measured by gyro)

  // compute uncertainty of the state prediction
  // uncertainty (= covariance P) is updated along with the state prediction
  // P = P + Q
  // Q is added to the uncertainty at each time step, the longer we rely on the prediction without correction, the higher the uncertainty grows
  KalUncer += (KalElapsedT * KalElapsedT) * 0.00487f;  // 4*4
  // standard deviation of rotation rate: 4(deg/s)
  // variance of rotation rate: std^2 (= estimation of the error on the gyro measurement)
  // accumulation of variance over a time period: variance of the angular velocity * time interval^2

  // compute Kalman gain: relative ratio of (uncertainty on the predicted state) / (uncertainty on the predicted state + uncertainty on the measurements)
  // K = P / (P + R)
  // K: controls how much trust is placed in the (acce) measurement vs. the prediction, determines how much measurement should influence the estimate
  float KalGain = KalUncer / (KalUncer + 0.00274f);  // 3*3
  // assumes that the (acce) measurement error has a Gaussian distribution with standard deviation = 3 degrees

  // update estimated state using the (acce) measurement
  KalState += KalGain * (KalMeas - KalState);  // (...): difference between measured angle and prediction of the angle
  
  // update uncertainty of the new predicted state, new uncertainty is reduced after incorporating measurement data
  KalUncer *= (1.0f - KalGain);

  // Kalman filter output
  Kalman1DOutput[0] = KalState;  // prediction for the state
  Kalman1DOutput[1] = KalUncer;  // corresponding uncertainty
}


void AttitudeEstimator::ImuAHRSUpdate(float elapsedTime, Q_t &_q) {
  float halfT;  // Half of delta time
  float norm;  // Normalization factor
  float vx, vy, vz;  // Estimated gravity directions
  float ex, ey, ez;  // Error terms
  // static volatile float ex_dt, ey_dt, ez_dt;  // Integral error terms for gyro bias correction
  static float ex_dt, ey_dt, ez_dt;  // Integral error terms for gyro bias correction
  float tempq0, tempq1, tempq2, tempq3;  // Temporary quaternion components

  // Precompute quaternion products for efficiency
  // float q0q0 = _q.q0 * _q.q0;
  // float q0q1 = _q.q0 * _q.q1;
  // float q0q2 = _q.q0 * _q.q2;
  // float q0q3 = _q.q0 * _q.q3;
  // float q1q1 = _q.q1 * _q.q1;
  // float q1q2 = _q.q1 * _q.q2;
  // float q1q3 = _q.q1 * _q.q3;
  // float q2q2 = _q.q2 * _q.q2;
  // float q2q3 = _q.q2 * _q.q3;
  // float q3q3 = _q.q3 * _q.q3;

  // volatile float gx, gy, gz, ax, ay, az;  // Gyro and accelerometer readings
  float gx, gy, gz, ax, ay, az;  // Gyro and accelerometer readings
  gx = Gyro_device[0] * 0.01745f;  // Convert gyro from deg/s to rad/s  // *PI/180
  gy = Gyro_device[1] * 0.01745f;
  gz = Gyro_device[2] * 0.01745f;
  ax = Acce_device[0];  // Accelerometer (in g)
  ay = Acce_device[1];
  az = Acce_device[2];

  halfT = elapsedTime * 0.5f;

  // Normalize accelerometer vector
  norm = invSqrt(ax * ax + ay * ay + az * az);  // Calculate normalization factor
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Estimated gravity vector based on quaternion
  vx = 2.0f * (_q.q1*_q.q3 - _q.q0*_q.q2);
  vy = 2.0f * (_q.q0*_q.q1 + _q.q2*_q.q3);
  vz = _q.q0*_q.q0 - _q.q1*_q.q1 - _q.q2*_q.q2 + _q.q3*_q.q3;

  // Cross-product error between measured gravity and estimated gravity
  ex = (ay * vz - az * vy);
  ey = (az * vx - ax * vz);
  ez = (ax * vy - ay * vx);

  if (ex != 0.0f && ey != 0.0f && ez != 0.0f) {
    // Integral terms: accumulate error over time for gyro drift bias
    ex_dt += ex * ki_ * halfT;
    ey_dt += ey * ki_ * halfT;
    ez_dt += ez * ki_ * halfT;
    // Correct gyro rates with proportional and integral terms
    gx += kp_ * ex + ex_dt;
    gy += kp_ * ey + ey_dt;
    gz += kp_ * ez + ez_dt;
  } 

  // // Store the corrected values
  // corrected_gyro[0] = gx;
  // corrected_gyro[1] = gy;
  // corrected_gyro[2] = gz;

  // Quaternion differential equations
  // Update the quaternion based on angular velocity
  tempq0 = _q.q0 + (-_q.q1 * gx - _q.q2 * gy - _q.q3 * gz) * halfT;
  tempq1 = _q.q1 + ( _q.q0 * gx + _q.q2 * gz - _q.q3 * gy) * halfT;
  tempq2 = _q.q2 + ( _q.q0 * gy - _q.q1 * gz + _q.q3 * gx) * halfT;
  tempq3 = _q.q3 + ( _q.q0 * gz + _q.q1 * gy - _q.q2 * gx) * halfT;

  // Normalize quaternion
  norm = invSqrt(tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3);  // Calculate normalization factor
  _q.q0 = tempq0 * norm;
  _q.q1 = tempq1 * norm;
  _q.q2 = tempq2 * norm;
  _q.q3 = tempq3 * norm;
}


void AttitudeEstimator::ConvertToEulerAngleByQ(EulerAngle_t &_EuAngle, const Q_t &_q) {  // const: avoid unnecessary copying, as the readings are only read, not modified.
  //  _EuAngle.yaw = -atan2f(2.0f * (_q.q0*_q.q2 + _q.q0*_q.q3), 1.0f - 2.0f * (_q.q2*_q.q2 - _q.q3*_q.q3));  // method1  // ZhaJiHu
  //  _EuAngle.yaw =  atan2f(2.0f * (_q.q1*_q.q2 + _q.q0*_q.q3), 1.0f - 2.0f * (_q.q2*_q.q2 - _q.q3*_q.q3));  // method2  // Orbital Mechanics for Engineering Students, Wikipedia
  //  _EuAngle.yaw =  atan2f(2.0f * (_q.q0*_q.q1 + _q.q2*_q.q3), 1.0f - 2.0f * (_q.q1*_q.q1 + _q.q2*_q.q2));  // method3  // Fabio Bobrow
   _EuAngle.yaw =  atan2f(2.0f * (_q.q1*_q.q2 + _q.q0*_q.q3), 1.0f - 2.0f * (_q.q2*_q.q2 + _q.q3*_q.q3));
   _EuAngle.pitch = asinf(2.0f * (_q.q0*_q.q2 - _q.q1*_q.q3));
  //  _EuAngle.roll = atan2f(2.0f * (_q.q2*_q.q3 + _q.q0*_q.q1), 1.0f - 2.0f * (_q.q1*_q.q1 - _q.q2*_q.q2));
  //  _EuAngle.roll = atan2f(2.0f * (_q.q0*_q.q3 + _q.q1*_q.q2), 1.0f - 2.0f * (_q.q2*_q.q2 + _q.q3*_q.q3));  // Fabio Bobrow
   _EuAngle.roll = atan2f(2.0f * (_q.q2*_q.q3 + _q.q0*_q.q1), 1.0f - 2.0f * (_q.q1*_q.q1 + _q.q2*_q.q2));
}


// Calculate Euler rates directly from the quaternion
void AttitudeEstimator::estimateAngularVelocityFromQ(float GyroRateEuler[3], const Q_t& _q) {
  // Get body rates (p, q, r) from the gyroscope in rad/sec.
  float p_rad = Gyro_device[0] * 0.01745f;  // Body roll rate  // *PI/180
  float q_rad = Gyro_device[1] * 0.01745f;  // Body pitch rate
  float r_rad = Gyro_device[2] * 0.01745f;  // Body yaw rate

  // --- Extract trigonometric terms directly from the quaternion ---
  
  // sin(pitch) can be calculated from the quaternion's pitch component definition
  float sinPitch = 2.0f * (_q.q0*_q.q2 - _q.q3*_q.q1);
  
  // Clamp to avoid issues from floating point inaccuracies
  if (sinPitch > 1.0f)  sinPitch = 1.0f;
  if (sinPitch < -1.0f) sinPitch = -1.0f;

  // cos(pitch) derived from sin(pitch) using sqrt(1 - sin^2)
  float cosPitch = sqrtf(1.0f - sinPitch * sinPitch);

  // The terms inside atan2 for roll give us proportional values for sin(roll) and cos(roll)
  float y_roll_term = 2.0f * (_q.q0*_q.q1 + _q.q2*_q.q3);
  float x_roll_term = 1.0f - 2.0f * (_q.q1*_q.q1 + _q.q2*_q.q2);
  
  // Normalize to get the actual sin(roll) and cos(roll)
  // This is faster than atan2 -> sin/cos
  float norm_roll = sqrtf(x_roll_term * x_roll_term + y_roll_term * y_roll_term);
  float sinRoll, cosRoll;

  if (norm_roll > 1e-6f) {
    sinRoll = y_roll_term / norm_roll;
    cosRoll = x_roll_term / norm_roll;
  } else {
    sinRoll = 0.0f;
    cosRoll = 1.0f;
  }
  
  // --- Handle Gimbal Lock ---
  // If cos(pitch) is near zero, we are at gimbal lock.
  if (fabsf(cosPitch) < 1e-6f) {
    // At gimbal lock, the transformation is singular.
    GyroRateEuler[2] = p_rad;  // Roll Rate (phi_dot)
    GyroRateEuler[1] = q_rad * cosRoll - r_rad * sinRoll;  // Pitch Rate (theta_dot)
    GyroRateEuler[0] = 0.0f;  // Yaw Rate (psi_dot) is undefined
    return;
  }

  // --- Apply Kinematic Transformation using the extracted terms ---
  float tanPitch = sinPitch / cosPitch;

  float phi_dot   = p_rad + q_rad * sinRoll * tanPitch + r_rad * cosRoll * tanPitch;
  float theta_dot =         q_rad * cosRoll            - r_rad * sinRoll;
  float psi_dot   =         q_rad * sinRoll / cosPitch + r_rad * cosRoll / cosPitch;

  // The output array `GyroRateEuler` expects {Yaw, Pitch, Roll} rates.
  GyroRateEuler[0] = psi_dot;
  GyroRateEuler[1] = theta_dot;
  GyroRateEuler[2] = phi_dot;
}


void AttitudeEstimator::complementaryQ(float dt, Q_t &_q) {
  float gx, gy, gz, ax, ay, az;  // Gyro and accelerometer readings
  gx = Gyro_device[0] * 0.01745f;  // Convert gyro from deg/s to rad/s
  gy = Gyro_device[1] * 0.01745f;
  gz = Gyro_device[2] * 0.01745f;
  ax = Acce_device[0];  // Accelerometer (in g)
  ay = Acce_device[1];
  az = Acce_device[2];

  // Predict the new orientation using gyroscope data.
  // Predict rotation quaternion time derivative.
  float q0_dot = 0.5f * (-_q.q1 * gx - _q.q2 * gy - _q.q3 * gz);
  float q1_dot = 0.5f * ( _q.q0 * gx - _q.q3 * gy + _q.q2 * gz);
  float q2_dot = 0.5f * ( _q.q3 * gx + _q.q0 * gy - _q.q1 * gz);
  float q3_dot = 0.5f * (-_q.q2 * gx + _q.q1 * gy + _q.q0 * gz);

  // Integrate to get the new predicted quaternion.
  _q.q0 += q0_dot * dt;
  _q.q1 += q1_dot * dt;
  _q.q2 += q2_dot * dt;
  _q.q3 += q3_dot * dt;

  // Correct the orientation using accelerometer data.
  // Normalize the accelerometer vector
  float a_norm = invSqrt(ax * ax + ay * ay + az * az);  // Calculate normalization factor
  ax *= a_norm;
  ay *= a_norm;
  az *= a_norm;

  // Only correct if the accelerometer reading is valid (not in freefall).
  if (a_norm > 0.0f) {
      // The estimated gravity vector in the body frame according to the current quaternion estimate.
      // This is essentially the third column of the rotation matrix.
      float vx = 2.0f * (_q.q1*_q.q3 - _q.q0*_q.q2);  // Estimated gravity directions
      float vy = 2.0f * (_q.q0*_q.q1 + _q.q2*_q.q3);
      float vz = _q.q0*_q.q0 - _q.q1*_q.q1 - _q.q2*_q.q2 + _q.q3*_q.q3;

      // The error is the cross product between the measured gravity (accelerometer) and the estimated gravity.
      float ex = (ay * vz - az * vy);  // Error terms
      float ey = (az * vx - ax * vz);
      float ez = (ax * vy - ay * vx);

      // Apply the correction using the PI controller gains (lds is the proportional gain here).
      // The reference uses a simplified correction that is mathematically similar to a gradient descent correction step.
      // Quaternion differential equations
      // Update the quaternion based on angular velocity
      _q.q0 += lds * dt * 0.5f * (-_q.q1 * ex - _q.q2 * ey - _q.q3 * ez);
      _q.q1 += lds * dt * 0.5f * ( _q.q0 * ex + _q.q2 * ez - _q.q3 * ey);
      _q.q2 += lds * dt * 0.5f * ( _q.q0 * ey - _q.q1 * ez + _q.q3 * ex);
      _q.q3 += lds * dt * 0.5f * ( _q.q0 * ez + _q.q1 * ey - _q.q2 * ex);
  }

  // Normalize the final quaternion to ensure it remains a unit quaternion
  float q_norm = invSqrt(_q.q0 * _q.q0 + _q.q1 * _q.q1 + _q.q2 * _q.q2 + _q.q3 * _q.q3);  // Calculate normalization factor
  _q.q0 *= q_norm;
  _q.q1 *= q_norm;
  _q.q2 *= q_norm;
  _q.q3 *= q_norm;
}


// Getter function definition
void AttitudeEstimator::getAcceDevice(float acce_data[3]) const {
    // Copy the private member data into the provided array
    // memcpy(acce_data, Acce_device, sizeof(Acce_device));

    // Normalize the accelerometer vector
    float norm = invSqrt(Acce_device[0] * Acce_device[0] + Acce_device[1] * Acce_device[1] + Acce_device[2] * Acce_device[2]);  // Calculate normalization factor
    acce_data[0] = Acce_device[0] * norm;
    acce_data[1] = Acce_device[1] * norm;
    acce_data[2] = Acce_device[2] * norm;
}