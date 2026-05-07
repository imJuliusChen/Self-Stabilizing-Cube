#ifndef ATTITUDE_TRAJECTORY_H
#define ATTITUDE_TRAJECTORY_H

#include <math.h>
#include "parameters.h"
#include <SimpleFOC.h>
#include "math_utils.h"

class AttitudeTrajectory {
public:
    AttitudeTrajectory();

    // Update the trajectory state
    void update(float dt);

    // Process raw audio signal for clap detection
    // void processAudioSignal(uint8_t mic_pin);
    void processAudioSignal(uint8_t mic_pin, uint8_t buzzer_pin);

    // Set the target rotational velocity (rad/s)
    // Positive for one direction, negative for the other, 0 to stop.
    void setTargetVelocity(float target_vel);

    // Reset everything to initial state (Yaw = 0)
    void reset();

    // Get the current modification quaternion (rotation around Z)
    // Multiplies the base reference quaternion by this to get the spinning reference.
    void getYawRotationQuaternion(float q_out[4]);

    // Check if the trajectory is currently active (velocity non-zero or moving)
    bool isActive();

private:
    float current_yaw;  // Current yaw angle (rad)
    float current_vel;  // Current angular velocity (rad/s)
    float target_vel;   // Target angular velocity (rad/s)
    const float max_accel = 1.0f;  // Acceleration limit for smooth transitions (rad/s^2)

    // Audio Processing & State Machine Variables
    enum ClapState {
        STATE_IDLE,
        STATE_WAIT_FOR_SECOND,
        STATE_SPINNING
    };

    ClapState clap_state;
    unsigned long state_timer;     // General purpose timer
    unsigned long debounce_timer;  // To prevent single loud noise triggering multiple claps
    float noise_floor;  // Dynamic Noise Variables  // Tracks background motor noise

    // Config
    // const int CLAP_THRESHOLD = 500;   // Amplitude threshold (adjust based on mic sensitivity)
    const int SPIKE_THRESHOLD = 500;  // How much louder than background a clap must be
    const int MIN_AMPLITUDE = 150;    // Hard minimum to ignore silence.
    
    const unsigned long DOUBLE_CLAP_WINDOW = 300000;  // us to wait for 2nd clap
    const unsigned long DEBOUNCE_TIME = 150000;       // us to ignore sound after a clap
    const float SPIN_SPEED = 0.1f;                    // rad/s
};

#endif