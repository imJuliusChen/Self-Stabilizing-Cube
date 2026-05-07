// Declares helper functions.

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <cmath>
#include <SimpleFOC.h>
#include "parameters.h"

void applyTransM(float input[3], float output[3], float TransM[3][3]);

void computeDCM(float DCM[3][3], float cpsi, float spsi, float ctheta, float stheta, float cphi, float sphi);

float constrainAngle(float x);

float Limit(float x, float lLimit, float uLimit);

float getSign(float val);

void beep(int pin, int duration);

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

float invSqrt(float x);

#endif