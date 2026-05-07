// Implements the LQR controller.

#include "lqr_controller.h"

LQRController::LQRController() {
  // memset(x_hat, 0, sizeof(x_hat));  // Initializes x_hat[3] = {0, 0, 0};
  // memset(theta_integral_sum, 0, sizeof(theta_integral_sum));  // Initialize integral term
}


// LQR stabilization controller function
// Discrete-Time LQR + Kalman Filter for Reaction Wheel Inverted Pendulum
// calculating the torque/voltage that needs to be set to the motor in order to stabilize the pendulum
/*
- Uses discrete-time LQR control gains (K) and Kalman filter gains (Kf) from MATLAB
- Observes theta and phi using sensors (e.g., encoders or IMU)
- Controls the motor torque to stabilize the pendulum
- Includes reference state tracking for proper stabilization
*/
void LQRController::control(float theta_meas[3], float thetadot_meas[3], float phidot_meas[3], 
  float theta_integral[3], float phidot_integral[3], float u[3], 
  const float K[3][3], const float Ki[3], const float Ki_vel[3], const float wr[3][3]) {
  // if angle is controllable, calculate the control law, LQR controller u = -k * x
  // Kalman Filter Full-State Estimation
  // x_k+1 = x_k + Kf * (y - C * x)

/*
  // Prediction Step
  float x_pred[3] = {0};
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          x_pred[i] += pgm_read_float(&Ad[i][j]) * x_hat[j];  // A_d * x_hat
      }
      x_pred[i] += pgm_read_float(&Bd[i][0]) * u_prev;  // + B_d * u_prev
  }

  // Update Step with Measurements
  float y[2] = {meas1, meas2};  // measured theta and phi_dot, should represent (2x1), but C arrays are row-major (1x2)
  float y_pred[2] = {0};
  for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 3; j++) {
          y_pred[i] += pgm_read_float(&Cd[i][j]) * x_pred[j];  // C_d * x_pred
      }
  }
  float innovation[2];
  for (int i = 0; i < 2; i++) {
      innovation[i] = y[i] - y_pred[i];  // y - C_d * x_pred
  }

  // State correction
  float correction[3] = {0};
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 2; j++) {
          correction[i] += pgm_read_float(&Kf[i][j]) * innovation[j];  // K_f * (y - C_d * x_pred)
      }
  }
  for (int i = 0; i < 3; i++) {
      x_hat[i] = x_pred[i] + correction[i];  // updated state
  }
*/

  // No Kalman filter
  // State vector assembly from measurements.
//   x_hat[0] = meas1;  x_hat[1] = meas2;  x_hat[2] = meas3;
  // x_hat[0][0] = theta_meas[0];  // Roll angle
  // x_hat[1][0] = theta_meas[1];  // Pitch angle
  // x_hat[2][0] = theta_meas[2];  // Yaw angle
  // x_hat[0][1] = thetadot_meas[0];  // Roll rate
  // x_hat[1][1] = thetadot_meas[1];  // Pitch rate
  // x_hat[2][1] = thetadot_meas[2];  // Yaw rate
  // x_hat[0][2] = phidot_meas[0];  // Motor 1 velocity
  // x_hat[1][2] = phidot_meas[1];  // Motor 2 velocity
  // x_hat[2][2] = phidot_meas[2];  // Motor 3 velocity

  // LQR control law: Compute LQR Control Input (Torque) with Reference Tracking
  // u = -K * (x - wr) - Ki * integral
  for (int i = 0; i < 3; i++) {
    u[i] = 0;  // Initializes control input

    // Compute state error inline
    float theta_error = theta_meas[i] - wr[i][0];
    float thetadot_error = thetadot_meas[i] - wr[i][1];
    float phidot_error = phidot_meas[i] - wr[i][2];

    u[i] = -(K[i][0] * theta_error +  // The core control law  // dot product
             K[i][1] * thetadot_error +
             K[i][2] * phidot_error +
             Ki[i] * theta_integral[i] +  // Add the integral term for steady-state angle error correction
             Ki_vel[i] * phidot_integral[i]);  // Add the integral term for wheel velocity
  }
  
  // Apply torque limits
  // u = constrain(u, -max_torque, max_torque);
  // u_prev = u;  // Store for next iteration

  // return u;
}


// Implementation of the quaternion-based control method
void LQRController::control_q(const float theta_error_body[3], float thetadot_meas[3], float phidot_meas[3],
    float theta_integral[3], float phidot_integral[3], float u[3],
    const float K[3][3], const float Ki[3], const float Ki_vel[3]) {

    // LQR control law using the body-frame error
    for (int i = 0; i < 3; i++) {
        u[i] = 0;  // Initialize control input

        u[i] = -(K[i][0] * theta_error_body[i] +
                 K[i][1] * thetadot_meas[i] +
                 K[i][2] * phidot_meas[i] +
                 Ki[i] * theta_integral[i] +
                 Ki_vel[i] * phidot_integral[i]);
    }
}