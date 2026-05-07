// Declares the manager for managing different balance modes.

#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include "parameters.h"
#include "math_utils.h"
#include <cmath>

// Enum to define the possible balancing modes for the device.
enum BalanceMode {
  INACTIVE,       // The device is not in a controllable position (e.g., fallen over).
  EDGE_BALANCE,   // Balancing on one of its edges.
  VERTEX_BALANCE  // Balancing on one of its vertices.
};

class BalanceModeManager {
public:
  // Cache computed values to avoid repeated calculations
  struct ModeCache {
    float ref_state[3][3];
    float K_gains[3][3];
    float Ki_gains[3];
    float Ki_vel_gains[3];
    float target_quaternion[4];
    bool active_motors[3];
    bool is_valid;
  };
  
  // Constructor
  BalanceModeManager();

  // Updates the current balance mode based on the measured angles.
  // theta_meas: An array containing the measured roll, pitch, and yaw angles in radians.
  void updateMode(float theta_meas[3]);

  // Gets the current balance mode.
  // return: The current BalanceMode enum value.
  BalanceMode getCurrentMode() const;

  /*
  // Populates a 2D array with the reference state for the current mode.
  // ref_state: A 3x3 array to be filled with the reference state [axis][state_var].
  void getReferenceState(float ref_state[3][3]);

  // Populates a boolean array indicating which motors are active for the current mode.
  // active_motors: A 3-element array to be filled (true if active, false otherwise).
  void getMotorActivation(bool active_motors[3]);

  // Populates a 2D array with the LQR gains for the current mode.
  // K_gains: A 3x3 array to be filled with the LQR gains.
  void getGains(float K_gains[3][3]);

  // Populates an array with the Ki gains for the current mode.
  void getKiGains(float Ki_gains[3]);

  // Populates an array with the Ki_vel gains for the current mode.
  void getKiVelGains(float Ki_vel_gains[3]);
  */

  // Fast parameter retrieval (just returns pointer to cached data)
  const ModeCache* getModeCache() const;

  // Setters
  void setVertexGain(int row, int col, float val);
  void setVertexKi(int axis, float val);
  void setVertexQuat(float q0, float q1, float q2, float q3);
  // Getter
  float getVertexGain(int row, int col) const;

private:
  BalanceMode currentMode;  // Stores the current active balance mode.

  // Caches for each mode
  ModeCache cache_edge;
  ModeCache cache_vertex;
  ModeCache cache_inactive;

  void initializeCaches();
};

#endif  // MODE_MANAGER_H