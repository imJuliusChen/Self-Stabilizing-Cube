// Implements helper functions.

#include "math_utils.h"

// apply transformation matrix to transform the expression of vector's component
void applyTransM(float input[3], float output[3], float TransM[3][3]) {
    for (int i = 0; i < 3; i++) {
        output[i] = TransM[i][0] * input[0] + TransM[i][1] * input[1] + TransM[i][2] * input[2];
    }
}


// compute direction cosine matrix (rotation matrix), using the yaw-pitch-roll sequence
// Euler angles are defined from inertial frame to body-fixed frame
// passive rotation: transform expression of vector's components
// transform from inertial frame to body-fixed frame
void computeDCM(float DCM[3][3], float cpsi, float spsi, float ctheta, float stheta, float cphi, float sphi) {
    // float psi = YAW_ANGLE * (PI / 180.0);
    // float theta = PITCH_ANGLE * (PI / 180.0);
    // float phi = ROLL_ANGLE * (PI / 180.0);

    // float cpsi = cos(psi), spsi = sin(psi);
    // float ctheta = cos(theta), stheta = sin(theta);
    // float cphi = cos(phi), sphi = sin(phi);

    DCM[0][0] = cpsi * ctheta;
    DCM[0][1] = spsi * ctheta;
    DCM[0][2] = -stheta;

    DCM[1][0] = cpsi * stheta * sphi - spsi * cphi;
    DCM[1][1] = spsi * stheta * sphi + cpsi * cphi;
    DCM[1][2] = ctheta * sphi;

    DCM[2][0] = cpsi * stheta * cphi + spsi * sphi;
    DCM[2][1] = spsi * stheta * cphi - cpsi * sphi;
    DCM[2][2] = ctheta * cphi;
}

/*
// compute kinematic mapping matrix (non-orthogonal), using the yaw-pitch-roll sequence
// Euler angles are defined from inertial frame to body-fixed frame
// passive rotation: transform expression of vector's components
// transform from body-fixed frame to Euler angles (yaw-pitch-roll sequence)
void computeKMM(float KMM[3][3], float PITCH_ANGLE, float ROLL_ANGLE) {
    float theta = PITCH_ANGLE * (PI / 180.0);
    float phi = ROLL_ANGLE * (PI / 180.0);

    float ctheta = cos(theta), ttheta = tan(theta);
    float cphi = cos(phi), sphi = sin(phi);

    KMM[0][0] = 0;
    KMM[0][1] = sphi / ctheta;
    KMM[0][2] = cphi / ctheta;

    KMM[1][0] = 0;
    KMM[1][1] = cphi;
    KMM[1][2] = -sphi;

    KMM[2][0] = 1;
    KMM[2][1] = sphi * ttheta;
    KMM[2][2] = cphi * ttheta;
}
*/

// constraining the angle in between -pi and pi (-180 and 180 degrees)
float constrainAngle(float x) {
  // Adding PI to x shifts the angle temporarily
  x = fmodf(x + PI, 2 * PI);  // remainder of (x + pi) / 2pi  // the result is in the range [0,2pi) (0 inclusive to 2pi exclusive)
  if (x < 0)
    x += 2 * PI;
  return x - PI;  // subtract pi from the result to shift the range from [0,2pi) to [-pi,pi), this gives the constrained angle centered around 0, which is the desired output range
}


float Limit(float x, float lLimit, float uLimit) {
  if (x <= lLimit) {  // Checks if data exceeds lower limit
    x = lLimit;  // Caps at lower limit
  }
  if (x >= uLimit) {  // Checks if data exceeds upper limit
    x = uLimit;  // Caps at upper limit
  }
  return x;
}


float getSign(float val) {
  return (val > 0) - (val < 0);  // Returns 1 if positive, -1 if negative, 0 if zero
}


// avoid timer conflict introduced by SimpleFOC
// avoid using tone() after SimpleFOC initialization. Instead, use a simple delay-based toggle to generate a buzzer sound
void beep(int pin, int duration) {
  digitalWrite(pin, HIGH);  // Turn buzzer on
  _delay(duration);          // Keep it on for the specified duration (ms)
  digitalWrite(pin, LOW);   // Turn buzzer off
}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) {
    return out_min;  // Avoid division by zero
  }
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Uses the Quake III Arena: fast inverse square root algorithm for efficient normalization of vectors and quaternions.
float invSqrt(float x) {
  float halfx = 0.5f * x;  // Compute half of input
  float y = x;  // Initial guess

  long i = *(long*)&y;  // Takes the float bits and "reinterprets" them as a long integer. It allows us to do integer math on the float's exponent, which gives a rough estimate of the square root in log space.
  i = 0x5f3759df - (i>>1);  // "Magic number" hack for initial approximation
  // 0x5f3759df is a magic constant derived empirically for 32-bit floats.
  // "i>>1": Shifts the bits to the right (equivalent to dividing the exponent by 2). A approximation of the square root in logarithmic space.
  // The subtraction transforms the log-sqrt to an approximate inverse sqrt.
  
  y = *(float*)&i;  // Reinterprets the modified integer i back into a float. Now y ≈ 1 / sqrt(x) (rough approximation).
  y = y * (1.5f - (halfx * y * y));  // One Newton-Raphson iteration to refine the result.
  return y;  // Return inverse square root
}