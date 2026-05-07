#include "attitude_trajectory.h"


AttitudeTrajectory::AttitudeTrajectory() {
    current_yaw = 0.0f;
    current_vel = 0.0f;
    target_vel = 0.0f;

    clap_state = STATE_IDLE;
    state_timer = 0;
    debounce_timer = 0;
    noise_floor = 150.0f;  // Initialize noise floor estimate
}


// Audio Processing with Peak-to-Peak Burst
// void AttitudeTrajectory::processAudioSignal(uint8_t mic_pin) {
void AttitudeTrajectory::processAudioSignal(uint8_t mic_pin, uint8_t buzzer_pin) {
    unsigned long now = _micros();

    // 1. BURST SAMPLING
    // Read the mic for ~2ms to find the wave height (Max - Min).    
    int signalMax = 0;
    int signalMin = 4096;
    unsigned long startMicros = _micros();
    
    // Sample for 2 milliseconds
    while (_micros() - startMicros < 2000) {
        int sample = analogRead(mic_pin);
        // if (sample < 1024) {  // Filter out spurious 0 readings common on ESP32
             if (sample > signalMax) signalMax = sample;
             if (sample < signalMin) signalMin = sample;
        // }
    }
    
    int amplitude = signalMax - signalMin;
    // Serial.print("Amplitude:"); Serial.println(amplitude);

    // 2. DYNAMIC NOISE FLOOR UPDATE
    // Smoothly track the background noise
    // 0.95 factor = adapts to new noise level in ~0.2 seconds
    noise_floor = (0.95f * noise_floor) + (0.05f * (float)amplitude);

    // 3. Clap Detection (Adaptive)
    bool clap_detected = false;
    
    // Trigger if:
    // A) Amplitude is above the hard minimum (silence gate) AND
    // B) Amplitude is significantly higher than the current background noise
    if (amplitude > MIN_AMPLITUDE && amplitude > (noise_floor + SPIKE_THRESHOLD)) {
        if (now - debounce_timer > DEBOUNCE_TIME) {
            clap_detected = true;
            debounce_timer = now;
            // Serial.println("Clap detected!");
        }
    }

    // 4. State Machine
    switch (clap_state) {
        case STATE_IDLE:
            // If stopped and clap detected -> Start waiting for potential 2nd clap
            if (clap_detected && fabs(target_vel) < 0.01f) {
                clap_state = STATE_WAIT_FOR_SECOND;
                state_timer = now;
            }
            // If already moving and clap detected -> STOP
            else if (clap_detected && fabs(target_vel) > 0.01f) {
                setTargetVelocity(0.0f);  // Stop
                clap_state = STATE_IDLE;
                // beep(buzzer_pin, 50);  // Short beep for Stop
            }
            break;

        case STATE_WAIT_FOR_SECOND:
            // Check for 2nd clap within window
            if (clap_detected) {
                // Double clap detected! -> Spin CCW
                setTargetVelocity(-SPIN_SPEED);
                clap_state = STATE_SPINNING;  // Go to spinning state
                // beep(buzzer_pin, 200);  // Long beep for CCW
            } 
            // Timeout expired? -> It was just a Single Clap -> Spin CW
            else if (now - state_timer > DOUBLE_CLAP_WINDOW) {
                setTargetVelocity(SPIN_SPEED);
                clap_state = STATE_SPINNING;
                // beep(buzzer_pin, 50);  // Short beep for CW
            }
            break;

        case STATE_SPINNING:
            // State to indicate we started via sound.
            if (clap_detected) {
                setTargetVelocity(0.0f);
                clap_state = STATE_IDLE;
                // beep(buzzer_pin, 50);  // Short beep for Stop
            }
            break;
    }
}


void AttitudeTrajectory::setTargetVelocity(float target) {
    target_vel = target;
}


void AttitudeTrajectory::reset() {
    current_yaw = 0.0f;
    current_vel = 0.0f;
    target_vel = 0.0f;
    clap_state = STATE_IDLE;
}


bool AttitudeTrajectory::isActive() {
    // Active if moving or if target is set to move
    return (fabs(current_vel) > 0.001f || fabs(target_vel) > 0.001f);
}


void AttitudeTrajectory::update(float dt) {
    // Smoothly ramp velocity to target
    if (current_vel < target_vel) {
        current_vel += max_accel * dt;
        if (current_vel > target_vel) current_vel = target_vel;
    } else if (current_vel > target_vel) {
        current_vel -= max_accel * dt;
        if (current_vel < target_vel) current_vel = target_vel;
    }

    // Integrate position
    current_yaw += current_vel * dt;

    // Normalize yaw to -PI to PI range (optional, but good practice)
    if (current_yaw > PI) current_yaw -= 2 * PI;
    if (current_yaw < -PI) current_yaw += 2 * PI;
}


void AttitudeTrajectory::getYawRotationQuaternion(float q_out[4]) {
    // Construct a quaternion representing rotation around the inertial Z-axis (vertical)
    // q_rot = [cos(0/2), ux*sin(0/2), uy*sin(0/2), uz*sin(0/2)] = [cos(yaw/2), 0, 0, sin(yaw/2)]
    float half_yaw = current_yaw * 0.5f;
    q_out[0] = cosf(half_yaw);
    q_out[1] = 0.0f;
    q_out[2] = 0.0f;
    q_out[3] = sinf(half_yaw);
}