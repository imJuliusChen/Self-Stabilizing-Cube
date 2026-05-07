// Declares the LQR controller.

#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include <cstring>  // for memset
#include "parameters.h"

class LQRController {
public:
  LQRController();
  void control(float theta_meas[3], float thetadot_meas[3], float phidot_meas[3], 
    float theta_integral[3], float phidot_integral[3], float u[3], 
    const float K[3][3], const float Ki[3], const float Ki_vel[3], const float wr[3][3]);
  void control_q(const float theta_error_body[3], float thetadot_meas[3], float phidot_meas[3],
    float theta_integral[3], float phidot_integral[3], float u[3],
    const float K[3][3], const float Ki[3], const float Ki_vel[3]);

private:
  // state variables  // Initial state estimate (1x3)
  // float x_hat[3][3];  // State estimate: [axis][state_var] -> [roll/pitch/yaw][theta/thetadot/phidot]
  // float theta_integral_sum[3];  // Accumulated integral of theta error
};

#endif