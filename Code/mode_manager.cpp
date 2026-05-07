// Implements the logic for the BalanceModeManager class.

#include "mode_manager.h"
#include <cstring>  // For memcpy

// Constructor: Initializes the manager to the INACTIVE state by default.
BalanceModeManager::BalanceModeManager() {
  currentMode = INACTIVE;
  initializeCaches();  // Pre-computes all caches
}


// Pre-loads all parameters into the cache structs
void BalanceModeManager::initializeCaches() {
    // Edge mode cache
    memcpy(cache_edge.ref_state, wr_edge, sizeof(wr_edge));
    memcpy(cache_edge.K_gains, K_edge, sizeof(K_edge));
    memcpy(cache_edge.Ki_gains, Ki_edge, sizeof(Ki_edge));
    memcpy(cache_edge.Ki_vel_gains, Ki_vel_edge, sizeof(Ki_vel_edge));
    cache_edge.target_quaternion[0] = qu0_EDGE;  // Initialization for Edge quaternion
    cache_edge.target_quaternion[1] = qu1_EDGE;
    cache_edge.target_quaternion[2] = qu2_EDGE;
    cache_edge.target_quaternion[3] = qu3_EDGE;
    cache_edge.active_motors[0] = true;
    cache_edge.active_motors[1] = false;
    cache_edge.active_motors[2] = false;
    cache_edge.is_valid = true;
    
    // Vertex mode cache
    memcpy(cache_vertex.ref_state, wr_vertex, sizeof(wr_vertex));
    memcpy(cache_vertex.K_gains, K_vertex, sizeof(K_vertex));
    memcpy(cache_vertex.Ki_gains, Ki_vertex, sizeof(Ki_vertex));
    memcpy(cache_vertex.Ki_vel_gains, Ki_vel_vertex, sizeof(Ki_vel_vertex));
    cache_vertex.target_quaternion[0] = qu0_VERTEX;  // Initialization for Vertex quaternion
    cache_vertex.target_quaternion[1] = qu1_VERTEX;
    cache_vertex.target_quaternion[2] = qu2_VERTEX;
    cache_vertex.target_quaternion[3] = qu3_VERTEX;
    cache_vertex.active_motors[0] = true;
    cache_vertex.active_motors[1] = true;
    cache_vertex.active_motors[2] = true;
    cache_vertex.is_valid = true;

    // Inactive mode cache
    memcpy(cache_inactive.ref_state, wr_zero, sizeof(wr_zero));
    memcpy(cache_inactive.K_gains, K_zero, sizeof(K_zero));
    memcpy(cache_inactive.Ki_gains, Ki_zero, sizeof(Ki_zero));
    memcpy(cache_inactive.Ki_vel_gains, Ki_zero, sizeof(Ki_zero));
    cache_inactive.target_quaternion[0] = 1.0f;  // Default Initialization
    cache_inactive.target_quaternion[1] = 0.0f;
    cache_inactive.target_quaternion[2] = 0.0f;
    cache_inactive.target_quaternion[3] = 0.0f;
    cache_inactive.active_motors[0] = false;
    cache_inactive.active_motors[1] = false;
    cache_inactive.active_motors[2] = false;
    cache_inactive.is_valid = true;
}


// Updates the current balance mode based on the device's orientation.
void BalanceModeManager::updateMode(float theta_meas[3]) {
  float roll = theta_meas[0];
  float pitch = theta_meas[1];
  // float yaw = theta_meas[2];  // Yaw is not used for mode switching for now.

  // Check for Vertex Balance: roll is ~40.5 deg, pitch is ~-40.5 deg
  // Check for Edge Balance: roll is ~45 deg, pitch is near 0
  // If none of the above, the cube has fallen over or is in transition.
  if (fabsf(pitch - VERTEX_TARGET_ANGLE_Y) < 2 * ANGLE_TOLERANCE) {
    if (fabsf(roll - (-VERTEX_TARGET_ANGLE_X)) < 2 * ANGLE_TOLERANCE) {
      currentMode = VERTEX_BALANCE;
    } else {
      currentMode = INACTIVE;
    }
  } else {
    if (fabsf(roll - (-EDGE_TARGET_ANGLE)) < ANGLE_TOLERANCE) {
      currentMode = EDGE_BALANCE;
    } else {
      currentMode = INACTIVE;
    }
  }
}


// Returns the currently active balance mode.
BalanceMode BalanceModeManager::getCurrentMode() const {
  return currentMode;
}


// Efficiently returns a pointer to the correct pre-loaded cache.
// Fast parameter retrieval (just returns pointer to cached data)
const BalanceModeManager::ModeCache* BalanceModeManager::getModeCache() const {
    switch (currentMode) {
      case EDGE_BALANCE:   return &cache_edge;
      case VERTEX_BALANCE: return &cache_vertex;
      case INACTIVE:
      default:             return &cache_inactive;
    }
}


/*
// Provides the reference state (wr) based on the current mode.
void BalanceModeManager::getReferenceState(float ref_state[3][3]) {
  switch (currentMode) {
    case EDGE_BALANCE:
      memcpy(ref_state, wr_edge, sizeof(wr_edge));
      break;
    case VERTEX_BALANCE:
      memcpy(ref_state, wr_vertex, sizeof(wr_vertex));
      break;
    case INACTIVE:
    default:
      // Default to the zero reference state to be safe, though it won't be used.
      memcpy(ref_state, wr_zero, sizeof(wr_zero));
      break;
  }
}


// Provides a boolean array indicating which motors should be active.
void BalanceModeManager::getMotorActivation(bool active_motors[3]) {
  switch (currentMode) {
    case EDGE_BALANCE:
      // Only motor 1 (roll) is active.
      active_motors[0] = true;
      active_motors[1] = false;
      active_motors[2] = false;
      break;
    case VERTEX_BALANCE:
      // All motors are active for vertex stabilization.
      active_motors[0] = true;
      active_motors[1] = true;
      active_motors[2] = true;
      // active_motors[2] = false;
      break;
    case INACTIVE:
    default:
      // No motors are active for inactive modes.
      active_motors[0] = false;
      active_motors[1] = false;
      active_motors[2] = false;
      break;
  }
}


// Provides the LQR gain matrix (K) based on the current mode.
void BalanceModeManager::getGains(float K_gains[3][3]) {
    switch (currentMode) {
        case EDGE_BALANCE:
            memcpy(K_gains, K_edge, sizeof(K_edge));
            break;
        case VERTEX_BALANCE:
            memcpy(K_gains, K_vertex, sizeof(K_vertex));
            break;
        case INACTIVE:
        default:
            // For non-controlled modes, use a zero-gain matrix.
            memcpy(K_gains, K_zero, sizeof(K_zero));
            break;
    }
}


// Provides the Ki gain array based on the current mode.
void BalanceModeManager::getKiGains(float Ki_gains[3]) {
    switch (currentMode) {
        case EDGE_BALANCE:
            memcpy(Ki_gains, Ki_edge, sizeof(Ki_edge));
            break;
        case VERTEX_BALANCE:
            memcpy(Ki_gains, Ki_vertex, sizeof(Ki_vertex));
            break;
        default:
            memcpy(Ki_gains, Ki_zero, sizeof(Ki_zero));
            break;
    }
}


// Provides the Ki_vel gain array based on the current mode.
void BalanceModeManager::getKiVelGains(float Ki_vel_gains[3]) {
    switch (currentMode) {
        case EDGE_BALANCE:
            memcpy(Ki_vel_gains, Ki_vel_edge, sizeof(Ki_vel_edge));
            break;
        case VERTEX_BALANCE:
            memcpy(Ki_vel_gains, Ki_vel_vertex, sizeof(Ki_vel_vertex));
            break;
        default:
            memcpy(Ki_vel_gains, Ki_zero, sizeof(Ki_zero));
            break;
    }
}
*/


void BalanceModeManager::setVertexGain(int row, int col, float val) {
    if (row >= 0 && row < 3 && col >= 0 && col < 3) {
        cache_vertex.K_gains[row][col] = val;
    }
}


void BalanceModeManager::setVertexKi(int axis, float val) {
    if (axis >= 0 && axis < 3) {
        cache_vertex.Ki_gains[axis] = val;
    }
}


void BalanceModeManager::setVertexQuat(float q0, float q1, float q2, float q3) {
    cache_vertex.target_quaternion[0] = q0;
    cache_vertex.target_quaternion[1] = q1;
    cache_vertex.target_quaternion[2] = q2;
    cache_vertex.target_quaternion[3] = q3;
}


float BalanceModeManager::getVertexGain(int row, int col) const {
    if (row >= 0 && row < 3 && col >= 0 && col < 3) {
        return cache_vertex.K_gains[row][col];
    }
    return 0.0f;
}