// Defines constants and gains.

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <pgmspace.h>  // ESP32-compatible pgmspace.h
#define PI 3.14159f

// const float MOTOR_R = 17.6f;     // Motor resistance in Ohms
// const float MOTOR_KE = 0.1061f;  // Back-EMF constant in V/(rad/s)
// const float MOTOR_KT = 0.1061f;  // Torque constant in N·m/A
// const float LQR_VOLTAGE_SCALE = 21.0f;  // Scales LQR effort to Volts  // (0.7*12)/0.4 = 21

// Inertia properties
// const float I_c_xx_bar = 0.005;  // In-plane moment of inertia (kgm^2)
// const float I_c_xy_bar = -0.0003;  // Out-of-plane moment of inertia (kgm^2)

/*
// declare Euler angles: sensor frame to desired device frame (yaw-pitch-roll)
// positive rotation: from sensor frame to desired device frame
// const float YawAngle_SensorToDevice PROGMEM = 0.0;  // degrees
// const float PitchAngle_SensorToDevice PROGMEM = 45.0 + 180.0;
// const float RollAngle_SensorToDevice PROGMEM = 4.5;
// const float cpsi PROGMEM = 1.0000f;  // cos(0.0 * PI/180)
// const float spsi PROGMEM = 0.0000f;  // sin(0.0 * PI/180)
const float cpsi PROGMEM = 0.9950f;  // cos((0.0 - 5.75) * PI/180)  // adjust installation error  // for x y decouple
const float spsi PROGMEM = -0.1002f;  // sin((0.0 - 5.75) * PI/180)  // adjust installation error
// const float ctheta PROGMEM = -0.7071f;  // cos((45.0 + 180.0) * PI/180)
// const float stheta PROGMEM = -0.7071f;  // sin((45.0 + 180.0) * PI/180)
const float ctheta PROGMEM = -0.7934f;  // cos((45.0 + 180.0 - 7.5) * PI/180)  // adjust installation error  // for y axis
const float stheta PROGMEM = -0.6088f;  // sin((45.0 + 180.0 - 7.5) * PI/180)  // adjust installation error
// const float cphi PROGMEM =  0.7604f;  // cos((4.5 - 45.0) * PI/180)
// const float sphi PROGMEM = -0.6494f;  // sin((4.5 - 45.0) * PI/180)
const float cphi PROGMEM =  0.7997f;  // cos((4.5 - 45.0 + 3.6) * PI/180)  // adjust installation error  // for x axis
const float sphi PROGMEM = -0.6004f;  // sin((4.5 - 45.0 + 3.6) * PI/180)  // adjust installation error
*/

// Constants for balance mode switching
// const float EDGE_TARGET_ANGLE = 0.7854f;  // 45 degrees
// const float EDGE_TARGET_ANGLE = 0.0f;  // 0 degrees
// const float VERTEX_TARGET_ANGLE_X = 0.7854f;  // 45 degrees
// const float VERTEX_TARGET_ANGLE_Y = 0.6155f;  // 35.26 degrees
const float ANGLE_TOLERANCE = 0.175f;  // approx 10 degrees, for mode detection window

// discrete-time sampling time: 25ms
// declare discrete-time system matrices (obtained from MATLAB)
/*
const float Ad[3][3] PROGMEM = {
    { 1.0290, 0.0252,      0},
    { 2.3339, 1.0290,      0},
    {-2.3339,-0.0290, 1.0000}
};
const float Bd[3][1] PROGMEM = {
    {  -0.0140},
    {  -1.1218},
    { 159.4213}
};
const float Cd[2][3] PROGMEM = {
    {1, 0, 0},
    {0, 0, 1}
};
*/

// declare discrete-time LQR control gains (obtained from MATLAB) (1x3)
// const float K[3] PROGMEM = {-15.2516, -1.5957, -0.0042};
// const float K[3] = {-10.0, -1.00, -0.0005};
// const float K[3] = {-10.0, -1.00, -0.007};
// const float K[3] PROGMEM = {-4.3842, -0.4562, -0.0000};  // phi dot modified by hand to 0

// Mode-Specific LQR Gains
// Gains for edge balancing
const float K_edge[3][3] = {
  { 8.0, 1.20, 0.008},  // Roll control:  [theta, theta dot, phi dot]
  { 0.0,  0.0,   0.0},  // Pitch control: (inactive)
  { 0.0,  0.0,   0.0}   // Yaw control:   (inactive)
};
// Gains for vertex balancing
// const float K_vertex[3][3] = {  // gravity vector
//   {11.0, 1.20, 0.005},  // Roll control:  [theta, theta dot, phi dot]
//   { 9.0, 1.20, 0.005},  // Pitch control: [theta, theta dot, phi dot]
//   { 9.0, 1.20, 0.005}   // Yaw control:   [theta, theta dot, phi dot]
// };
const float K_vertex[3][3] = {  // fixed in space
  {12.0, 1.00, 0.005},  // Roll control:  [theta, theta dot, phi dot]
  { 9.0, 1.00, 0.005},  // Pitch control: [theta, theta dot, phi dot]
  { 9.0, 1.00, 0.005}   // Yaw control:   [theta, theta dot, phi dot]
};
// Zero-gain matrix for non-controlled modes
const float K_zero[3][3] = {
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0}
};

// declare discrete-time Kalman filter gains (obtained from MATLAB) (3x2)
// const float Kf[3][2] PROGMEM = {{0.5627, -0.1060}, {0.7886, -1.0337}, {-0.1060, 0.8228}};

// Declare integral gain for theta error
// const float Ki = -0.5;
const float Ki_edge[3]   = {0.5,  0.0,  0.0};  // Roll, Pitch, Yaw
// const float Ki_vertex[3] = {1.0, 1.0, 1.0};  // Roll, Pitch, Yaw  // gravity vector
const float Ki_vertex[3] = {0.5, 0.5, 0.5};  // Roll, Pitch, Yaw  // fixed in space
const float Ki_zero[3]   = {0.0, 0.0, 0.0};  // Zero-gain arrays for inactive modes

// Declare integral gain for phidot_axis error
// const float Ki_vel = -0.005;
const float Ki_vel_edge[3]   = {0.0, 0.0, 0.0};
// const float Ki_vel_vertex[3] = {0.0001, 0.0001, 0.0001};  // Roll, Pitch, Yaw
const float Ki_vel_vertex[3] = {0.000, 0.000, 0.000};  // Roll, Pitch, Yaw

// Gain for the attitude estimator's complementary filter
// This determines how much to trust the accelerometer vs. the gyroscope.
const float lds = 1.0f;

// declare reference state (desired equilibrium position)
// In control theory the state is typically (3x1). However C/C++ arrays are row-major (1x3)
// const float wr[3] = {0, 0, 0};  // Upright pendulum with zero velocities
// [theta, theta dot, phi, phi dot]
// theta    : Pendulum angle from the vertical (rad)
// theta dot: Angular velocity of the pendulum (rad/s)
// (canceled)phi      : Absolute angular position of the reaction wheel (rad)
// phi dot  : Angular velocity of the reaction wheel (rad/s)
/*
// Default reference state with zero velocities
const float wr_zero[3][3] = {
  {0, 0, 0},  // Roll axis
  {0, 0, 0},  // Pitch axis
  {0, 0, 0}   // Yaw axis
};
// Reference state for edge balancing
const float wr_edge[3][3] = {
  {-EDGE_TARGET_ANGLE - 0.04f, 0, 0},  // Roll axis
  {0, 0, 0},                           // Pitch axis
  {0, 0, 0}                            // Yaw axis
};
// Reference state for vertex balancing
const float wr_vertex[3][3] = {
  {-VERTEX_TARGET_ANGLE_X + 0.058f, 0, 0},  // Roll axis
  { VERTEX_TARGET_ANGLE_Y - 0.045f, 0, 0},  // Pitch axis
  {0, 0, 0}                                // Yaw axis
};
*/

// define system parameters
const float max_torque = 0.4;  // Maximum torque limit (N*m)


// Initial reference quaternions

// Target for balancing on an edge (Roll: -45 deg, Pitch: 0 deg, Yaw: 0 deg)
// Sitting on x axis edge, corrected for center of mass misalignment
const float qu0_EDGE =  0.92388f;
const float qu1_EDGE = -0.38268f;
const float qu2_EDGE =      0.0f;
const float qu3_EDGE =      0.0f;

// Target for balancing on a vertex (Euler angles in 3-2-1 sequence, Roll: -45+0.7 deg, Pitch: 35.26-2.1 deg, Yaw: -15 deg)
// Sitting on vertex, corrected for center of mass misalignment
const float qu0_VERTEX =  0.89413f;
const float qu1_VERTEX = -0.32376f;
const float qu2_VERTEX =  0.30923f;
const float qu3_VERTEX = -0.00918f;

// const float qu0_VERTEX =  1.0f;
// const float qu1_VERTEX =  0.0f;
// const float qu2_VERTEX =  0.0f;
// const float qu3_VERTEX =  0.0f;

#endif