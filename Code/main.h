// The main program initializes hardware, schedules tasks, and runs the control loop.

// --- MASTER DEBUG TOGGLE ---
#define DEBUG_SERIAL 0  // Set to 1 to enable serial printing, 0 to disable
// Defines custom functions 
#if DEBUG_SERIAL == 1
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else  // Replace any call to DEBUG_PRINT() with absolutely nothing
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

// Aggregates all headers.
#include <Wire.h>  // include Wire library
#include <SimpleFOC.h>

// Include utilities
#include "parameters.h"
#include "pin_names.h"
#include "math_utils.h"

// Include drivers
#include "mpu6050.h"
#include "magnetic_sensor.h"
#include "bldc_motor.h"

// Include modules
// #include "attitude_estimator.h"  // DCM + Kalman
#include "attitude_estimator_q.h"  // DCM + Kalman  // Quaternions + AHRS
#include "lqr_controller.h"
#include "mode_manager.h"
#include "attitude_trajectory.h"

// #include "esp_timer.h"  // For ESP32 hardware timer functions

// FreeRTOS headers for multi-tasking
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>  // Declares functions like xTaskCreatePinnedToCore() for task creation.
// #include <freertos/semphr.h>  // Provides semaphore and mutex support.

// MPU6050 Pins
#define MPU_SDA_PIN 4
#define MPU_SCL_PIN 5

// Mic Pin (Use an Analog Input pin)
#define MIC_PIN 34


// Objects declaration (global)
MPU6050 mpu6050(&Wire1);
AttitudeEstimator att_est;
LQRController cont;
BalanceModeManager mode_manag;
AttitudeTrajectory att_traj;
Commander command = Commander(Serial);  // Instantiate the Commander object, linked to Serial input
// MagneticSensor sensor;
MagneticSensor sensors[3] = {
  MagneticSensor(0x06, 14, 0x03, 8, &Wire),  // Sensor 0: MT6701 on Wire (Bus 0)
  MagneticSensor(0x36, 12, 0x0E, 4, &Wire),  // Sensor 1: AS5600 on Wire (Bus 0)
  MagneticSensor(0x36, 12, 0x0E, 4, &Wire1)  // Sensor 2: AS5600 on Wire1 (Bus 1, Shared with MPU6050)
};
// BLDCMotorController motor;
BLDCMotorController motors[3] = {
  BLDCMotorController(MOTOR1_PWM_A, MOTOR1_PWM_B, MOTOR1_PWM_C, MOTOR1_ENABLE),
  BLDCMotorController(MOTOR2_PWM_A, MOTOR2_PWM_B, MOTOR2_PWM_C, MOTOR2_ENABLE),
  BLDCMotorController(MOTOR3_PWM_A, MOTOR3_PWM_B, MOTOR3_PWM_C, MOTOR3_ENABLE)
};


// State variables
// Central data structure to share state between tasks safely
struct SystemState {
  // Sensor readings
  float theta_meas[3] = {0};       // Roll, pitch, yaw (rad)
  float thetadot_meas[3] = {0};    // Roll rate, pitch rate, yaw rate (rad/s)
  float phidot_meas[3] = {0};      // Motor velocities (rad/s)
  float phidot_filtered[3] = {0};  // Motor velocities (rad/s)
  // float phidot_axis[3] = {0};      // Motor velocities in xyz axes (rad/s)
  // float corrected_gyro[3] = {0};   // Corrected gyro rates (rad/s)
  float acce_sensor[3] = {0};  // Raw Acce_sensor readings
  // float acce_device[3] = {0};  // Acce_device readings
  
  // Monitor specific angle calculations
  // float AngleRoll_Kalman = 0;       // Roll angle from the Kalman filter (rad)
  // float AnglePitch_Kalman = 0;      // Pitch angle from the Kalman filter (rad)
  // float AngleRoll_Quaternion = 0;   // Roll angle from the AHRS/Quaternion filter (rad)
  // float AnglePitch_Quaternion = 0;  // Pitch angle from the AHRS/Quaternion filter (rad)
  // float AngleYaw_Quaternion = 0;    // Yaw angle from the AHRS/Quaternion filter (rad)

  // Control variables
  float u[3] = {0};  // Control input (torque)
  // float u_axis[3] = {0};  // Control input (torque)
  // float u_prev = 0;  // Previous control input
  float target_voltage[3] = {0};  // Control output
  // float feedback_voltage[3] = {0};
  // float feedforward_voltage[3] = {0};
  float theta_integral[3] = {0};  // Integral of theta error
  float theta_error[3] = {0};
  float phidot_integral[3] = {0};  // Integral of phidot error

  float q_meas[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // Current orientation quaternion  // w, x, y, z
  // float q_meas2[4] = {1.0f, 0.0f, 0.0f, 0.0f};
};


// State variables
float GyroCaliX = 0, GyroCaliY = 0, GyroCaliZ = 0;  // define calibrated gyroscope variables  // global variables


// uint32_t LoopTimer;  // unsigned 32-bit integer
// unsigned long previousTime;  // unsigned 32-bit integer


// Safety and state flags
// bool flag_arm = false;
// bool flag_terminate = false;
float max_voltage = motors[0].getMaxVoltage();  // voltage limit set to the motor


// Serial control toggle
bool serial_control_enabled = false;  // Toggle for enabling/disabling manual control via Serial

// Serial command forward declarations
void doTargetVoltage(char* cmd);  // Serial command for motor voltage
void doToggleSerial(char* cmd);  // Serial command to toggle serial control
// void doDebugSensors(char* cmd);
void doAcceAverage(char* cmd);  // Serial command for acce averaging

void doSetVertexK(char* cmd);   // Serial command to set K_vertex
void doIncVertexK(char* cmd);   // Serial command to inc/dec K_vertex
void doSetVertexKi(char* cmd);  // Serial command to set Ki_vertex
void doSetVertexQu(char* cmd);  // Serial command to set Qu_vertex
void doSpin(char* cmd);


class VelocityFilter {
private:
  float prev_value;
  float prev_filtered;
  // static constexpr float spike_threshold = 10.0f;  // rad/s  // tune based on the system
  // static constexpr float alpha = 0.50f;  // Low-pass coefficient
  float spike_threshold;  // rad/s  // tune based on the system
  float alpha;  // Low-pass coefficient
  
public:
  // VelocityFilter() : prev_value(0), prev_filtered(0) {}
  VelocityFilter(float threshold = 10.0f, float lpf_alpha = 0.5f) : prev_value(0), prev_filtered(0), spike_threshold(threshold), alpha(lpf_alpha) {}
  
  // Single-pass filter: spike rejection + low-pass in one step
  float filter(float raw_value) {
    // Spike rejection: if change is too large, it's likely noise
    float delta = raw_value - prev_value;
    if (fabsf(delta) > spike_threshold) { raw_value = prev_value; }  // Treat as outlier, use previous value
    
    // Low-pass filter (exponential moving average)
    float filtered = alpha * prev_filtered + (1.0f - alpha) * raw_value;
    
    // Update history
    prev_value = raw_value;
    prev_filtered = filtered;
    
    return filtered;
  }
  
  void reset() { prev_value = 0; prev_filtered = 0; }
};


/*
// Timer and interrupt variables
volatile bool controlUpdateFlag = false;  // Flag to indicate when control update is needed (set by ISR, read in loop)
// volatile tells the compiler that this variable can be changed outside normal program flow (i.e. by an ISR).
esp_timer_handle_t control_timer_handle;  // A handle to manage the ESP32 timer object
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;  // A spinlock/mutex to protect shared variables (used in ISRs)

void IRAM_ATTR onControlTimerWrapper(void* arg);
void initControlTimer();  // Initialize hardware timer
void executeControlUpdate(float elapsed, float a_meas[3], float g_meas[3]);  // Forward declaration
*/


// Global instance of the shared state and a mutex to protect it
SystemState sharedState;  // Global struct shared between tasks
SemaphoreHandle_t stateMutex;  // RTOS mutex (binary semaphore) to protect sharedState from concurrent access
// SemaphoreHandle_t i2cMutex;

// RTOS Task function forward declarations
void EstimationTask(void *pvParameters);     // Runs on Core 1, High Priority (100Hz)
void ControlTask(void *pvParameters);        // Runs on Core 1, High Priority (100Hz)
void motorFocTask(void *pvParameters);       // Runs on Core 0, High Priority
#if DEBUG_SERIAL == 1
void serialMonitorTask(void *pvParameters);  // Runs on Core 1, Low Priority
#endif



void setup() {

  pinMode(LED_BUILTIN, OUTPUT);  // ESP32 built-in LED (GPIO 2 typically)
  // digitalWrite(LED_BUILTIN, HIGH);  // turn on the internal LED
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    _delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    _delay(200);
  }
  Serial.begin(115200);  // start serial communication
  Serial.flush();  // flush the serial buffer, clear any residual data

  pinMode(BUZZER_PIN, OUTPUT);  // Set buzzer pin as output
  pinMode(MIC_PIN, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);  // Bus 0 (Sensors 0 & 1)
  Wire.setClock(400000);
  Wire1.begin(MPU_SDA_PIN, MPU_SCL_PIN);  // Bus 1 (Sensor 2 & MPU6050)
  Wire1.setClock(400000);
  mpu6050.init();

  Serial.println("Starting MPU-6050 calibration...");
  // Beep: Starting MPU-6050 calibration
  beep(BUZZER_PIN, 10);  // Single 10 ms beep

  mpu6050.calibrate(GyroCaliX, GyroCaliY, GyroCaliZ);  // Update global variables (pass by reference)

  // Beep: Calibration complete
  beep(BUZZER_PIN, 10);  // Single 10 ms beep

  // compute the fixed DCM once
  att_est.init();

  // Beep: Calibration complete
  // beep(BUZZER_PIN, 10);  // Single 10 ms beep


  // enable more verbose output for debugging (comment out if not needed)
  SimpleFOCDebug::enable(&Serial);

  // Sensor configuration
  // Initialize sensors and motors
  for (int i = 0; i < 3; i++) {
    // Initialize magnetic sensor, prepares the I2C interface and configures the sensor hardware
    // sensor.init();
    sensors[i].init();

    // Link the motor to the sensor
    // motor.linkSensor(&sensor);
    motors[i].linkSensor(&sensors[i]);
    
    // motor.init();
    motors[i].init();
  }


  // Initialize commander  // Register commands
  command.add('T', doTargetVoltage, "set individual motor voltage: T [index] [voltage]");  // Register T command: set voltage for motor
  command.add('S', doToggleSerial, "toggle serial control");  // Register S command: switch control modes
  // command.add('D', doDebugSensors, "debug sensor readings");
  command.add('a', doAcceAverage, "average Acce_sensor readings (10s)");

  command.add('K', doSetVertexK, "Set K_vertex: K [row] [col] [val]");
  command.add('k', doIncVertexK, "Inc/Dec K_vertex: k [row][col][+/-]");
  command.add('I', doSetVertexKi, "Set Ki_vertex: I [axis] [val]");
  command.add('Q', doSetVertexQu, "Set Qu_vertex: Q [q0] [q1] [q2] [q3]");
  command.add('V', doSpin, "Set spin velocity: V [rad/s]");  // Usage: V 0.5 (spin left), V -0.5 (spin right), V 0 (stop)


  // initControlTimer();  // Initialize the timer interrupt  // The RTOS scheduler replaces the hardware timer.


  // Create the mutex before creating tasks that use it
  stateMutex = xSemaphoreCreateMutex();
  if (stateMutex == NULL) {
    Serial.println("Mutex creation failed!");
    while(1);  // Halts the system if mutex creation fails
  }
  // i2cMutex = xSemaphoreCreateMutex();  // Create the mutex

  // Create the RTOS tasks
  // Arduino tasks (loop, serial) run on Core 1 by default.
  // CORE 1 TASKS (Real-Time Control & Monitoring)
  xTaskCreatePinnedToCore(
    EstimationTask,    // Task function to run
    "EstimationTask",  // Name of the task
    8192,              // Stack size in words
    NULL,              // Task input parameter
    3,                 // Priority of the task (higher number is higher priority)
    NULL,              // Task optional handle
    1);                // Core where the task should run

  xTaskCreatePinnedToCore(ControlTask, "ControlTask", 4096, NULL, 4, NULL, 1);

  #if DEBUG_SERIAL == 1  // Only create the task if debugging is enabled
  // Create the serial monitoring task with the lowest priority
  xTaskCreatePinnedToCore(serialMonitorTask, "SerialMonitorTask", 8192, NULL, 1, NULL, 1);
  #endif

  // CORE 0 TASK (Motor Commutation)
  xTaskCreatePinnedToCore(motorFocTask, "MotorFocTask", 4096, NULL, 2, NULL, 0); 


  // Delay to adjust the device angle (e.g., 5 seconds)
  // Serial.println("Delay start...");
  // _delay(2000);  // 2 seconds to position the pendulum near upright

  // Serial.println("Delay complete, playing tone...");
  // Beep: Entering loop
  beep(BUZZER_PIN, 10);  // Single 10 ms beep
  _delay(200);
  beep(BUZZER_PIN, 10);  // Single 10 ms beep

  // Serial.println("Setup complete, entering loop...");
  Serial.println("Setup complete, Dual-core RTOS tasks started...");
  Serial.println("Serial commands: T [index] [voltage], S to toggle serial control");

  // previousTime = _micros();
}



// MAIN LOOP (CORE 1) - Only for non-critical tasks
void loop() {
  // static unsigned long loop_previousTime = 0;
  // unsigned long loop_currenTime = _micros();
  // DEBUG_PRINT(" Loop interval (us):"); DEBUG_PRINTLN(loop_currenTime - loop_previousTime);
  // loop_previousTime = loop_currenTime;


  /*
  // Check if control update is needed (triggered by timer interrupt)
  // This avoids executing control logic multiple times per interrupt
  portENTER_CRITICAL(&timerMux);  // Enter critical section (non-ISR version)  // Protect shared flag from simultaneous ISR access
  bool needsUpdate = controlUpdateFlag;  // Copy flag to local variable
  if (needsUpdate) {
    controlUpdateFlag = false;  // Reset flag so we don’t execute again until next timer interrupt
  }
  portEXIT_CRITICAL(&timerMux);  // Exit critical section

  // Execute control logic only when timer interrupt triggers (every period)
  // If ISR set the flag, perform control update
  if (needsUpdate) {
    unsigned long currentTime = _micros();  // Restart the counter
    float elapsedTime = (currentTime - previousTime) / 1000000.0;  // elapsed time in seconds
    // DEBUG_PRINT(" elapsedTime (us):"); DEBUG_PRINTLN(currentTime - previousTime);
    previousTime = currentTime;
    
    executeControlUpdate(elapsedTime, Acce_sensor, Gyro_sensor);  // Run the entire control logic
  }
  */


  // SLOW PATH (Runs every 50ms): UI, Serial, State Management
  static unsigned long last_slow_update = 0;
  unsigned long now = _micros();
  if (now - last_slow_update > 50000) {  // --- START OF SLOW LOGIC ---
  last_slow_update = now;

  // Handle serial commands (User communication)
  command.run();  // checks if any new Serial input arrived and runs the registered handler


  // Determine the current balance mode
  Q_t local_q;
  EulerAngle_t local_euler;
  float local_theta_meas[3];
  bool gotData = false;

  if (xSemaphoreTake(stateMutex, (TickType_t)2) == pdTRUE) {
     local_q.q0 = sharedState.q_meas[0];
     local_q.q1 = sharedState.q_meas[1];
     local_q.q2 = sharedState.q_meas[2];
     local_q.q3 = sharedState.q_meas[3];
     gotData = true;
     xSemaphoreGive(stateMutex);
  }

  if (gotData) {
     att_est.ConvertToEulerAngleByQ(local_euler, local_q);  // Perform the expensive math
     // --- Pendulum angle (radians) (Quaternions + AHRS) ---
     local_theta_meas[0] = constrainAngle(local_euler.roll);
     local_theta_meas[1] = constrainAngle(local_euler.pitch);
     local_theta_meas[2] = constrainAngle(local_euler.yaw);

     // Determine the current balance mode and get its parameters (reference state, gain, motor activation)
     mode_manag.updateMode(local_theta_meas);
     
     if (xSemaphoreTake(stateMutex, (TickType_t)2) == pdTRUE) {
        memcpy(sharedState.theta_meas, local_theta_meas, sizeof(local_theta_meas));
        xSemaphoreGive(stateMutex);
     }
  }

  }  // --- END OF SLOW LOGIC ---


  // Read Microphone and Update Logic
  // int mic_val = analogRead(MIC_PIN); att_traj.processAudioSignal(mic_val);  // v1
  att_traj.processAudioSignal(MIC_PIN, BUZZER_PIN);  // v2 v3


  // _delay(5);  // in milliseconds
  vTaskDelay(pdMS_TO_TICKS(10));  // Don't hog the core
  // vTaskDelay(pdMS_TO_TICKS(5));
  // vTaskDelay(pdMS_TO_TICKS(20));
  // vTaskDelay(pdMS_TO_TICKS(50));
}



// CORE 0 TASK: MOTOR FOC AND MOVE
void motorFocTask(void *pvParameters) {
  float local_target_voltage[3] = {0};

  for(;;) {
    // Safely get the latest target voltage from Core 1's calculation
    if (xSemaphoreTake(stateMutex, (TickType_t)5) == pdTRUE) {
      // Wait only up to 5 RTOS ticks (≈ 5 ms if 1 kHz tick rate). If the mutex isn’t available within that time, return pdFALSE and the task continues without access to the shared state.
      // This is a non-blocking / timeout style.
      // Use short timeout (pdMS_TO_TICKS(x)) when timing matters more than freshness (better to miss one update than freeze the task).
      memcpy(local_target_voltage, sharedState.target_voltage, sizeof(local_target_voltage));
      xSemaphoreGive(stateMutex);
    }
    

    // main FOC algorithm function, the faster you run this function the better, Arduino UNO loop  ~1kHz
    // iterative function updating the sensor internal variables
    // it is usually called in motor.loopFOC()
    // this function reads the sensor hardware and has to be called before getAngle and getVelocity
    // ~1ms
    // motor.loopFOC();

    // IMPORTANT - call as frequently as possible
    // update/read the sensor values 
    // sensor.update();

    for (int i = 0; i < 3; i++) {
      // PROTECT I2C ACCESS
      // if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        // Run the FOC algorithm as fast as possible for all motors
        // sensors[i].selectMuxChannel();  // Good practice to select channel before any motor interaction
        // sensors[i].update();
        motors[i].loopFOC();

        // xSemaphoreGive(i2cMutex);
      // }
    }
    for (int i = 0; i < 3; i++) {
      // Move all motors based on the latest command
      // Motion control function (Set the target voltage to the motor)
      // this function can be run at much lower frequency than loopFOC() function
      // motor.move(target_voltage);
      // motor.move(30);
      motors[i].move(local_target_voltage[i]);
    }

    // No delay needed, this task should run as fast as possible on Core 0.
  }
}



// CORE 1 TASK: HIGH-FREQUENCY ESTIMATION (100Hz)
// Task for reading sensors and estimating attitude
void EstimationTask(void *pvParameters) {
  static unsigned long previousTime = _micros();

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  // Run this task at 100 Hz
  xLastWakeTime = xTaskGetTickCount();

  // Task-Local Variables
  float Gyro_sensor[3];  // declare the global gyroscope readings variables  // sequence: X, Y, Z
  float Acce_sensor[3];  // declare the global accelerometer readings variables  // sequence: X, Y, Z
  float GyroRateEuler[3];  // sequence: Yaw, Pitch, Roll
  // float local_corrected_gyro[3];
  float local_theta_meas[3], local_thetadot_meas[3], local_phidot_meas[3];
  float local_phidot_filtered[3] = {0};  // motor 1, 2, 3 velocities
  // float local_phidot_axis[3] = {0};  // x, y, z velocities (variables for axis-space control)
  // static VelocityFilter vel_filters[3];  // One filter per motor
  static VelocityFilter vel_filters[3] = {  // Motor Velocity Filters
    VelocityFilter(10.0f, 0.5f),
    VelocityFilter(10.0f, 0.5f),
    VelocityFilter(10.0f, 0.5f)
  };
  static VelocityFilter thetadot_filters[3] = {  // Gyro Velocity Filters
    VelocityFilter(2.0f, 0.1f),
    VelocityFilter(2.0f, 0.1f),
    VelocityFilter(2.0f, 0.1f)
  };
  float local_q_meas[4];
  // float local_q_meas2[4];
  // float local_acce_device[3];

  // define predicted angles and uncertainties coming from Kalman filter
  // float AngleRoll_Kal = 0, KalUncerAngleRoll = 4.0;
  // float AnglePitch_Kal = 0, KalUncerAnglePitch = 4.0;
  // initial guess (value is not important)
  // 1. state (= angle) = 0
  // 2. uncertainty (= variance = std^2), standard deviation = 2 degrees


  for(;;) {  // Infinite loop for the task
    // Dynamically calculate elapsedTime for THIS task
    unsigned long currentTime = _micros();  // Restart the counter
    float elapsedTime = (currentTime - previousTime) / 1000000.0f;  // elapsed time in seconds
    // DEBUG_PRINT(" elapsedTime (us):"); DEBUG_PRINTLN(currentTime - previousTime);
    previousTime = currentTime;
    
    // Read all hardware sensors
    // call the predefined function to read the gyro measurements
    // if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      mpu6050.readSensors(Gyro_sensor, Acce_sensor, GyroCaliX, GyroCaliY, GyroCaliZ);
      // xSemaphoreGive(i2cMutex);
    // }


    // Perform attitude estimation
    // Update internal Gyro_device and Acce_device
    att_est.updateDeviceReadings(Gyro_sensor, Acce_sensor);
    // att_est.getAcceDevice(local_acce_device);

    // Estimate attitude and states (DCM + Kalman)
    // att_est.estimate(elapsedTime, AngleRoll_Kal, AnglePitch_Kal, KalUncerAngleRoll, KalUncerAnglePitch, GyroRateEuler);
    // Estimate attitude and states (Quaternions + AHRS)
    att_est.ImuAHRSUpdate(elapsedTime, att_est.q_);
    local_q_meas[0] = att_est.q_.q0;
    local_q_meas[1] = att_est.q_.q1;
    local_q_meas[2] = att_est.q_.q2;
    local_q_meas[3] = att_est.q_.q3;
    // att_est.ConvertToEulerAngleByQ(att_est.EuAngle_, att_est.q_);

    // att_est.complementaryQ(elapsedTime, att_est.q_2);
    // local_q_meas2[0] = att_est.q_2.q0;
    // local_q_meas2[1] = att_est.q_2.q1;
    // local_q_meas2[2] = att_est.q_2.q2;
    // local_q_meas2[3] = att_est.q_2.q3;
    // local_q_meas[0] = att_est.q_2.q0;
    // local_q_meas[1] = att_est.q_2.q1;
    // local_q_meas[2] = att_est.q_2.q2;
    // local_q_meas[3] = att_est.q_2.q3;

    // Estimate angular velocity (radians/s)
    att_est.estimateAngularVelocity(GyroRateEuler);
    // att_est.estimateAngularVelocityFromQ(GyroRateEuler, att_est.q_);


    // --- Pendulum angle (radians) (DCM + Kalman) ---
    // local_theta_meas[0] = constrainAngle(AngleRoll_Kal);
    // local_theta_meas[1] = constrainAngle(AnglePitch_Kal);
    // --- Pendulum angle (radians) (Quaternions + AHRS) ---
    // local_theta_meas[0] = constrainAngle(att_est.EuAngle_.roll);
    // local_theta_meas[1] = constrainAngle(att_est.EuAngle_.pitch);
    // local_theta_meas[2] = constrainAngle(att_est.EuAngle_.yaw);

    // --- Pendulum angular velocity (radians/s) ---
    // local_thetadot_meas[0] = GyroRateEuler[2];
    // local_thetadot_meas[1] = GyroRateEuler[1];
    // local_thetadot_meas[2] = -GyroRateEuler[0];
    local_thetadot_meas[0] = thetadot_filters[0].filter(GyroRateEuler[2]);
    local_thetadot_meas[1] = thetadot_filters[1].filter(GyroRateEuler[1]);
    local_thetadot_meas[2] = thetadot_filters[2].filter(-GyroRateEuler[0]);

    // --- Wheel angle (radians) ---
    // float phi_meas = constrainAngle(motor.shaft_angle);
    // float phi_meas = constrainAngle(sensor.getAngle());

    // --- Wheel velocity (radians/s) ---
    // for (int i = 0; i < 3; i++) {
    //   sensors[i].selectMuxChannel();  // Good practice to select channel before any motor interaction
    //   local_phidot_meas[i] = -motors[i].getShaftVelocity();
    // }
    // sensors[0].selectMuxChannel();  // Good practice to select channel before any motor interaction
    local_phidot_meas[0] = -motors[0].getShaftVelocity();
    // sensors[1].selectMuxChannel();  // Good practice to select channel before any motor interaction
    local_phidot_meas[1] = -motors[1].getShaftVelocity();
    // sensors[2].selectMuxChannel();  // Good practice to select channel before any motor interaction
    local_phidot_meas[2] = -motors[2].getShaftVelocity();
    // float phidot_meas = -sensor.getVelocity();

    // Apply low-pass filter to motor velocity to reduce noise
    // local_phidot_filtered[0] = 0.95f * local_phidot_filtered[0] + 0.05f * local_phidot_meas[0];
    // local_phidot_filtered[1] = 0.95f * local_phidot_filtered[1] + 0.05f * local_phidot_meas[1];
    // local_phidot_filtered[2] = 0.95f * local_phidot_filtered[2] + 0.05f * local_phidot_meas[2];
    // Apply efficient filtering
    local_phidot_filtered[0] = vel_filters[0].filter(local_phidot_meas[0]);
    local_phidot_filtered[1] = vel_filters[1].filter(local_phidot_meas[1]);
    local_phidot_filtered[2] = vel_filters[2].filter(local_phidot_meas[2]);


    // Update the Global Shared State
    // Lock the mutex ONCE, copy all results, and immediately unlock.
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
      // Wait forever until the mutex becomes available. The task will be blocked indefinitely (not consuming CPU, but paused).
      // The safest choice when the task must update shared state, no matter how long it takes.
      // Use portMAX_DELAY when correctness matters more than timing (must get the data).
      // Update sensor state for monitoring
      memcpy(sharedState.q_meas, local_q_meas, sizeof(local_q_meas));
      // memcpy(sharedState.q_meas2, local_q_meas2, sizeof(local_q_meas2));
      // memcpy(sharedState.theta_meas, local_theta_meas, sizeof(local_theta_meas));
      memcpy(sharedState.thetadot_meas, local_thetadot_meas, sizeof(local_thetadot_meas));
      // memcpy(sharedState.phidot_meas, local_phidot_meas, sizeof(local_phidot_meas));
      memcpy(sharedState.phidot_filtered, local_phidot_filtered, sizeof(local_phidot_filtered));
      // memcpy(sharedState.phidot_axis, local_phidot_axis, sizeof(local_phidot_axis));
      // memcpy(sharedState.corrected_gyro, att_est.corrected_gyro, sizeof(att_est.corrected_gyro));
      memcpy(sharedState.acce_sensor, Acce_sensor, sizeof(Acce_sensor));
      // memcpy(sharedState.acce_device, local_acce_device, sizeof(local_acce_device));

      // Update specific angle calculations for monitoring
      // sharedState.AngleRoll_Kalman = AngleRoll_Kal;
      // sharedState.AnglePitch_Kalman = AnglePitch_Kal;
      // sharedState.AngleRoll_Quaternion = att_est.EuAngle_.roll;
      // sharedState.AnglePitch_Quaternion = att_est.EuAngle_.pitch;
      // sharedState.AngleYaw_Quaternion = att_est.EuAngle_.yaw;

      xSemaphoreGive(stateMutex);  // Release the mutex
    }

    // Serial.printf("Task %s HighWaterMark: %d\n", pcTaskGetName(NULL), uxTaskGetStackHighWaterMark(NULL));
    
    // Convert ms to RTOS ticks
    // vTaskDelay(pdMS_TO_TICKS(10));  // Run this task at ~100 Hz
    // vTaskDelay(pdMS_TO_TICKS(5));  // Run this task at ~200 Hz
    // vTaskDelay(pdMS_TO_TICKS(2));  // Run this task at ~500 Hz
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}



// CORE 1 TASK: LQR CONTROLLER (100Hz)
// Task for calculating control output
void ControlTask(void *pvParameters) {
  static unsigned long previousTimeC = _micros();

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  // Run this task at 100 Hz
  xLastWakeTime = xTaskGetTickCount();

  BalanceMode previous_mode = INACTIVE;

  // Task-Local Variables
  float local_theta_meas[3], local_thetadot_meas[3];
  // float local_phidot_meas[3];
  float local_phidot_filtered[3];
  // float local_phidot_axis[3];
  // float local_u_axis[3] = {0};  // x, y, z torques (from LQR)
  float local_u[3] = {0}, local_target_voltage[3] = {0};  // Torques for motor 1, 2, 3
  // static float smoothed_target_voltage[3] = {0};
  // float local_feedback_voltage[3] = {0}, local_feedforward_voltage[3] = {0};
  float local_theta_integral[3] = {0};
  float local_theta_error[3] = {0};
  float local_phidot_integral[3] = {0};
  float local_q_meas[4];


  for(;;) {  // Infinite loop for the task
    // Dynamically calculate elapsedTime for THIS task
    unsigned long currentTimeC = _micros();  // Restart the counter
    float elapsedTimeC = (currentTimeC - previousTimeC) / 1000000.0f;  // elapsed time in seconds
    // DEBUG_PRINT(" elapsedTimeC (us):"); DEBUG_PRINTLN(currentTimeC - previousTimeC);
    previousTimeC = currentTimeC;

    // Get Latest State from Estimator
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
      memcpy(local_q_meas, sharedState.q_meas, sizeof(local_q_meas));
      // memcpy(local_theta_meas, sharedState.theta_meas, sizeof(local_theta_meas));
      memcpy(local_thetadot_meas, sharedState.thetadot_meas, sizeof(local_thetadot_meas));
      // memcpy(local_phidot_meas, sharedState.phidot_meas, sizeof(local_phidot_meas));
      memcpy(local_phidot_filtered, sharedState.phidot_filtered, sizeof(local_phidot_filtered));
      // memcpy(local_phidot_axis, sharedState.phidot_axis, sizeof(local_phidot_axis));
      xSemaphoreGive(stateMutex);
    }


    // Control logic
    // loop down-sampling
    // control loop each period
    if (serial_control_enabled) {
      // Manual serial control is active, LQR is paused, do nothing here.
      memset(local_theta_integral, 0, sizeof(local_theta_integral));  // Reset the integral term to prevent windup when switching back.
      memset(local_phidot_integral, 0, sizeof(local_phidot_integral));  // Reset the integral term to prevent windup when switching back.
      digitalWrite(LED_BUILTIN, (_micros() / 100000) % 2);  // Fast blink for serial mode


    } else {
      // Determine the current balance mode and get its parameters (reference state, gain, motor activation)
      // mode_manag.updateMode(local_theta_meas);
      BalanceMode current_mode = mode_manag.getCurrentMode();

      // Single call to get all parameters (no memcpy overhead)
      const auto* mode_params = mode_manag.getModeCache();
      
      
      // Check for a mode change
      if (current_mode != previous_mode) {
        memset(local_theta_integral, 0, sizeof(local_theta_integral));  // Reset integral on any mode switch.
        memset(local_phidot_integral, 0, sizeof(local_phidot_integral));  // Reset integral on any mode switch.
        
        if (current_mode == VERTEX_BALANCE) {
            att_traj.reset();  // Ensures target always snaps back to the original (Yaw=0) position.
        } else {
            att_traj.setTargetVelocity(0.0f);  // Stop spinning if we leave Vertex mode
        }
        previous_mode = current_mode;  // Update the previous mode tracker.
      }


      // Update Trajectory
      if (current_mode == VERTEX_BALANCE) {
          att_traj.update(elapsedTimeC);
      }


      // Get the correct target quaternion
      // float qr0, qr1, qr2, qr3;
      // if (current_mode == VERTEX_BALANCE) {
      //     qr0 = qu0_VERTEX; qr1 = qu1_VERTEX; qr2 = qu2_VERTEX; qr3 = qu3_VERTEX;
      // } else {  // EDGE or INACTIVE
      //     qr0 = qu0_EDGE; qr1 = qu1_EDGE; qr2 = qu2_EDGE; qr3 = qu3_EDGE;
      // }
      float qr0 = mode_params->target_quaternion[0];
      float qr1 = mode_params->target_quaternion[1];
      float qr2 = mode_params->target_quaternion[2];
      float qr3 = mode_params->target_quaternion[3];


      // For Trajectory
      // Apply Yaw Rotation to Reference Quaternion
      // q_new = q_rot_z * q_ref
      // Left Multiplication: rotates the object around the Global / Fixed Axes.
      if (current_mode == VERTEX_BALANCE) {
          float q_rot[4];
          att_traj.getYawRotationQuaternion(q_rot);
          
          float q_static_0 = qr0; float q_static_1 = qr1;
          float q_static_2 = qr2; float q_static_3 = qr3;

          // Quaternion multiplication
          qr0 = q_rot[0] * q_static_0 - q_rot[1] * q_static_1 - q_rot[2] * q_static_2 - q_rot[3] * q_static_3;
          qr1 = q_rot[0] * q_static_1 + q_rot[1] * q_static_0 + q_rot[2] * q_static_3 - q_rot[3] * q_static_2;
          qr2 = q_rot[0] * q_static_2 - q_rot[1] * q_static_3 + q_rot[2] * q_static_0 + q_rot[3] * q_static_1;
          qr3 = q_rot[0] * q_static_3 + q_rot[1] * q_static_2 - q_rot[2] * q_static_1 + q_rot[3] * q_static_0;
      }


      float q0 = local_q_meas[0]; float q1 = local_q_meas[1];
      float q2 = local_q_meas[2]; float q3 = local_q_meas[3];



      // Error Calculation Method 1: Fixed Target in 3D Space
      // Calculate error quaternion q_e = conjugate(q_current) * q_target
      // This gives the rotation from the current frame to the target, expressed in the body-frame.
      float qe0 = q0 * qr0 + q1 * qr1 + q2 * qr2 + q3 * qr3;
      float qe1 = q0 * qr1 - q1 * qr0 - q2 * qr3 + q3 * qr2;
      float qe2 = q0 * qr2 + q1 * qr3 - q2 * qr0 - q3 * qr1;
      float qe3 = q0 * qr3 - q1 * qr2 + q2 * qr1 - q3 * qr0;

      // Normalize rotation quaternion error
      float norm = invSqrt(qe0*qe0 + qe1*qe1 + qe2*qe2 + qe3*qe3);
      // qe0 *= norm;
      qe1 *= norm;
      qe2 *= norm;
      qe3 *= norm;

      // The vector part of the error quaternion (qe1, qe2, qe3) is proportional to the angle error around each body axis.
      // For small angles, angle_error_axis ≈ 2 * vector_part.
      local_theta_error[0] = -2.0f * qe1;  // Error around body X-axis (Roll)
      local_theta_error[1] = -2.0f * qe2;  // Error around body Y-axis (Pitch)
      local_theta_error[2] =  2.0f * qe3;  // Error around body Z-axis (Yaw)
      // local_theta_error[0] = -2.0f * asin(qe1);  // Error around body X-axis (Roll)
      // local_theta_error[1] = -2.0f * asin(qe2);  // Error around body Y-axis (Pitch)
      // local_theta_error[2] =  2.0f * asin(qe3);  // Error around body Z-axis (Yaw)
      // local_theta_error[0] = local_theta_meas[0] - mode_params->ref_state[0][0];
      // local_theta_error[1] = local_theta_meas[1] - mode_params->ref_state[1][0];
      // local_theta_error[2] = local_theta_meas[2] - mode_params->ref_state[2][0];


/*
      // Error Calculation Method 2: Gravity Vector Alignment
      // 1. Calculate Target Gravity Vector (Down direction in Body Frame)
      // Derived from the target quaternion in parameters.h (qr)
      // This vector is fixed as long as the mode doesn't change.
      float vg_x_target = 2.0f * (qr1 * qr3 - qr0 * qr2);
      float vg_y_target = 2.0f * (qr0 * qr1 + qr2 * qr3);
      float vg_z_target = qr0 * qr0 - qr1 * qr1 - qr2 * qr2 + qr3 * qr3;

      // 2. Calculate Current Gravity Vector
      // Derived from the real-time sensor quaternion (local_q_meas)      
      float vg_x_curr = 2.0f * (q1 * q3 - q0 * q2);
      float vg_y_curr = 2.0f * (q0 * q1 + q2 * q3);
      float vg_z_curr = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

      // 3. Calculate Error Vector (Cross Product: Target x Current)
      // This creates an error vector perpendicular to the tilt direction.
      // It inherently IGNORES rotation around the gravity vector (Yaw).
      local_theta_error[0] = -(vg_y_target * vg_z_curr - vg_z_target * vg_y_curr);
      local_theta_error[1] = -(vg_z_target * vg_x_curr - vg_x_target * vg_z_curr);
      local_theta_error[2] =  (vg_x_target * vg_y_curr - vg_y_target * vg_x_curr);
      // Note: This error is in the Body Frame, which matches the LQR controller expectation.
*/

      if (current_mode != INACTIVE) {
        // Update integral of theta error, ONLY for the reference angle of the current mode
        // This prevents integral windup on axes that aren't being controlled to zero
        local_theta_integral[0] += local_theta_error[0] * elapsedTimeC;  // Accumulate theta error over time
        local_theta_integral[1] += local_theta_error[1] * elapsedTimeC;
        local_theta_integral[2] += local_theta_error[2] * elapsedTimeC;

        local_theta_integral[0] *= 0.95;  // Apply decay
        local_theta_integral[1] *= 0.95;
        local_theta_integral[2] *= 0.95;
        
        // Anti-Windup (Clamping)
        // Limit the integral term so it doesn't saturate the motors completely just on its own
        // float integral_limit = 0.3f;  // Adjust this limit (in radians-ish) as needed
        // local_theta_integral[0] = Limit(local_theta_integral[0], -integral_limit, integral_limit);
        // local_theta_integral[1] = Limit(local_theta_integral[1], -integral_limit, integral_limit);
        // local_theta_integral[2] = Limit(local_theta_integral[2], -integral_limit, integral_limit);
        

        local_phidot_integral[0] += local_phidot_filtered[0] * elapsedTimeC;  // Accumulate phidot error over time
        local_phidot_integral[1] += local_phidot_filtered[1] * elapsedTimeC;
        local_phidot_integral[2] += local_phidot_filtered[2] * elapsedTimeC;

        local_phidot_integral[0] *= 0.95;  // Apply decay
        local_phidot_integral[1] *= 0.95;
        local_phidot_integral[2] *= 0.95;
      }


      // Calculate control torques using mode-specific gains and reference state
      // If mode is INACTIVE, the gains will be zero, so the torque 'u' will naturally become zero.
      if (current_mode == VERTEX_BALANCE) {
        // cont.control(local_theta_meas, local_thetadot_meas, local_phidot_axis, 
        //              local_theta_integral, local_phidot_integral, local_u, 
        //              mode_params->K_gains, mode_params->Ki_gains, mode_params->Ki_vel_gains, mode_params->ref_state);  // LQR now uses axis velocities and outputs axis torques
        cont.control_q(local_theta_error, local_thetadot_meas, local_phidot_filtered,
                       local_theta_integral, local_phidot_integral, local_u,
                       mode_params->K_gains, mode_params->Ki_gains, mode_params->Ki_vel_gains);

      } else {
        // cont.control(local_theta_meas, local_thetadot_meas, local_phidot_filtered, 
        //              local_theta_integral, local_phidot_integral, local_u, 
        //              mode_params->K_gains, mode_params->Ki_gains, mode_params->Ki_vel_gains, mode_params->ref_state);
        cont.control_q(local_theta_error, local_thetadot_meas, local_phidot_filtered,
                       local_theta_integral, local_phidot_integral, local_u,
                       mode_params->K_gains, mode_params->Ki_gains, mode_params->Ki_vel_gains);
        // cont.control(local_theta_meas, local_thetadot_meas, local_phidot_meas, 
        //              local_theta_integral, local_phidot_integral, local_u, 
        //              mode_params->K_gains, mode_params->Ki_gains, mode_params->Ki_vel_gains, mode_params->ref_state);
      }


      // Convert torques to voltages (motor command) and apply motor activation logic
      for (int i = 0; i < 3; i++) {
        /*
        // Apply voltage only if motor is active for this mode
        local_target_voltage[i] = mode_params->active_motors[i] ? Limit(mapFloat(local_u[i], -max_torque, max_torque, -max_voltage, max_voltage), -max_voltage, max_voltage) : 0;  // Apply limits
        */


        float feedback_voltage = 0;
        // float feedforward_voltage = 0;
    
        if (mode_params->active_motors[i]) {
            // 1. Feedback (LQR) Term: The voltage needed to produce torque
            feedback_voltage = mapFloat(local_u[i], -max_torque, max_torque, -max_voltage, max_voltage);

            // 2. Feedforward (BEMF) Term: The voltage needed to counteract BEMF
            // V_bemf = Ke * speed, the motor speed is -local_phidot_filtered[i] (due to direction)
            // feedforward_voltage = MOTOR_KE * (-local_phidot_filtered[i]);
            // feedforward_voltage = MOTOR_KE * (-local_phidot_filtered[i]) * 0.5f;

            // 3. (Optional) Dead-Zone Compensation
            // If the motor is trying to move, give it a minimum "push"
            // const float MIN_VOLTAGE = 0.2;  // Tune this! Find the lowest voltage that makes the motor spin
            // if (fabs(feedback_voltage) > 0.01 && fabs(feedback_voltage) < MIN_VOLTAGE) {
            //     feedback_voltage = getSign(feedback_voltage) * MIN_VOLTAGE;
            // }

        } else {
            feedback_voltage = 0;
            // feedforward_voltage = 0;
        }
        // The final voltage is the sum of both terms
        // local_target_voltage[i] = Limit(feedback_voltage + feedforward_voltage, -max_voltage, max_voltage);
        local_target_voltage[i] = Limit(feedback_voltage, -max_voltage, max_voltage);


        // smooth the output with a low-pass filter
        // Static variable to retain the smoothed voltage between iterations
        // it is declared static so it retains its value between loop() iterations, allowing continuous smoothing, it gets allocated for the lifetime of the program
        // smoothed_target_voltage[i] = 0.3f * smoothed_target_voltage[i] + 0.7f * local_target_voltage[i];  // Adjust coefficient for responsiveness
        // smoothed_target_voltage[i] = 0.5f * smoothed_target_voltage[i] + 0.5f * local_target_voltage[i];  // Adjust coefficient for responsiveness
        // smoothed_target_voltage[i] = 0.7f * smoothed_target_voltage[i] + 0.3f * local_target_voltage[i];  // Adjust coefficient for responsiveness
        // local_target_voltage[i] = smoothed_target_voltage[i];
      }
      

      // Update the Global Shared State
      // Lock the mutex ONCE, copy all results, and immediately unlock.
      if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
        // Update control state
        // memcpy(sharedState.u, local_u, sizeof(local_u));
        // memcpy(sharedState.u_axis, local_u_axis, sizeof(local_u_axis));
        // memcpy(sharedState.theta_integral, local_theta_integral, sizeof(local_theta_integral));
        memcpy(sharedState.target_voltage, local_target_voltage, sizeof(local_target_voltage));
        // memcpy(sharedState.feedback_voltage, local_feedback_voltage, sizeof(local_feedback_voltage));
        // memcpy(sharedState.feedforward_voltage, local_feedforward_voltage, sizeof(local_feedforward_voltage));
        // memcpy(sharedState.theta_error, local_theta_error, sizeof(local_theta_error));
        // memcpy(sharedState.phidot_integral, local_phidot_integral, sizeof(local_phidot_integral));
        xSemaphoreGive(stateMutex);  // Release the mutex
      }

      digitalWrite(LED_BUILTIN, (current_mode == EDGE_BALANCE || current_mode == VERTEX_BALANCE));  // LED feedback based on the mode
    }

    // Serial.printf("Task %s HighWaterMark: %d\n", pcTaskGetName(NULL), uxTaskGetStackHighWaterMark(NULL));
    
    // Convert ms to RTOS ticks
    // vTaskDelay(pdMS_TO_TICKS(20));  // Run this task at ~50 Hz
    // vTaskDelay(pdMS_TO_TICKS(10));  // Run this task at ~100 Hz
    // vTaskDelay(pdMS_TO_TICKS(5));  // Run this task at ~200 Hz
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}



#if DEBUG_SERIAL == 1
// Dedicated task for printing data to the serial monitor
void serialMonitorTask(void *pvParameters) {
  // Local copies of shared state variables to minimize mutex lock time
  // float local_AngleRoll_Kalman, local_AnglePitch_Kalman;
  // float local_AngleRoll_Quaternion, local_AnglePitch_Quaternion, local_AngleYaw_Quaternion;
  float local_theta_meas[3], local_thetadot_meas[3];
  float local_phidot_meas[3], local_phidot_filtered[3], local_phidot_axis[3];
  float local_theta_integral[3], local_phidot_integral[3];
  float local_u[3], local_u_axis[3], local_target_voltage[3];
  // float local_feedback_voltage[3], local_feedforward_voltage[3];
  float local_theta_error[3];
  // float local_corrected_gyro[3];
  float local_q_meas[4];
  // float local_q_meas2[4];
  float local_acce_sensor[3];
  // float local_acce_device[3];

  for(;;) {  // Infinite loop for the task
    // Lock the mutex and safely copy all data.
    // This ensures that no two tasks are modifying the same data at the same time (prevents race conditions).
    // xSemaphoreTake(...) blocks until it can safely access the shared data.  // xSemaphoreGive(...) releases the lock.
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
      // Get a consistent snapshot of the state
      // local_AngleRoll_Kalman = sharedState.AngleRoll_Kalman;
      // local_AnglePitch_Kalman = sharedState.AnglePitch_Kalman;
      // local_AngleRoll_Quaternion = sharedState.AngleRoll_Quaternion;
      // local_AnglePitch_Quaternion = sharedState.AnglePitch_Quaternion;
      // local_AngleYaw_Quaternion = sharedState.AngleYaw_Quaternion;

      // memcpy(local_q_meas, sharedState.q_meas, sizeof(local_q_meas));
      // memcpy(local_q_meas2, sharedState.q_meas2, sizeof(local_q_meas2));
      // memcpy(local_theta_meas, sharedState.theta_meas, sizeof(local_theta_meas));
      // memcpy(local_thetadot_meas, sharedState.thetadot_meas, sizeof(local_thetadot_meas));
      // memcpy(local_corrected_gyro, sharedState.corrected_gyro, sizeof(local_corrected_gyro));
      // memcpy(local_phidot_meas, sharedState.phidot_meas, sizeof(local_phidot_meas));
      // memcpy(local_phidot_filtered, sharedState.phidot_filtered, sizeof(local_phidot_filtered));
      // memcpy(local_phidot_axis, sharedState.phidot_axis, sizeof(local_phidot_axis));
      // memcpy(local_acce_sensor, sharedState.acce_sensor, sizeof(local_acce_sensor));
      // memcpy(local_acce_device, sharedState.acce_device, sizeof(local_acce_device));

      // memcpy(local_theta_integral, sharedState.theta_integral, sizeof(local_theta_integral));
      // memcpy(local_theta_error, sharedState.theta_error, sizeof(local_theta_error));
      // memcpy(local_phidot_integral, sharedState.phidot_integral, sizeof(local_phidot_integral));
      // memcpy(local_u_axis, sharedState.u_axis, sizeof(local_u_axis));
      // memcpy(local_u, sharedState.u, sizeof(local_u));
      // memcpy(local_target_voltage, sharedState.target_voltage, sizeof(local_target_voltage));
      // memcpy(local_feedback_voltage, sharedState.feedback_voltage, sizeof(local_feedback_voltage));
      // memcpy(local_feedforward_voltage, sharedState.feedforward_voltage, sizeof(local_feedforward_voltage));
      
      xSemaphoreGive(stateMutex);  // Release the mutex immediately after copying.
    }

    // Print the copied data. Do NOT hold the mutex during these slow operations. This part is slow but now it doesn't block other tasks.
    // Uncomment for debugging
    // Minimize Serial usage to save memory!!!!!
    // DEBUG_PRINT(" Roll_K(deg):"); DEBUG_PRINT(local_AngleRoll_Kalman * 57.2958f);
    // DEBUG_PRINT(" Pitch_K(deg):"); DEBUG_PRINT(local_AnglePitch_Kalman * 57.2958f);
    // DEBUG_PRINT(" Roll_Q(deg):"); DEBUG_PRINT(local_AngleRoll_Quaternion * 57.2958f);
    // DEBUG_PRINT(" Pitch_Q(deg):"); DEBUG_PRINT(local_AnglePitch_Quaternion * 57.2958f);
    // DEBUG_PRINT(" Yaw_Q(deg):"); DEBUG_PRINT(local_AngleYaw_Quaternion * 57.2958f);

    // DEBUG_PRINT(" q0_meas:"); DEBUG_PRINT(local_q_meas[0], 4);
    // DEBUG_PRINT(" q1_meas:"); DEBUG_PRINT(local_q_meas[1], 4);
    // DEBUG_PRINT(" q2_meas:"); DEBUG_PRINT(local_q_meas[2], 4);
    // DEBUG_PRINT(" q3_meas:"); DEBUG_PRINT(local_q_meas[3], 4);
    // DEBUG_PRINT(" q0_meas2:"); DEBUG_PRINT(local_q_meas2[0], 4);
    // DEBUG_PRINT(" q1_meas2:"); DEBUG_PRINT(local_q_meas2[1], 4);
    // DEBUG_PRINT(" q2_meas2:"); DEBUG_PRINT(local_q_meas2[2], 4);
    // DEBUG_PRINT(" q3_meas2:"); DEBUG_PRINT(local_q_meas2[3], 4);
    // DEBUG_PRINT(" theta_meas1(deg):"); DEBUG_PRINT(local_theta_meas[0] * 57.2958f);
    // DEBUG_PRINT(" theta_meas2(deg):"); DEBUG_PRINT(local_theta_meas[1] * 57.2958f);
    // DEBUG_PRINT(" theta_meas3(deg):"); DEBUG_PRINT(local_theta_meas[2] * 57.2958f);
    // DEBUG_PRINT(" thetadot_meas1(deg/s):"); DEBUG_PRINT(local_thetadot_meas[0] * 57.2958f);
    // DEBUG_PRINT(" thetadot_meas2(deg/s):"); DEBUG_PRINT(local_thetadot_meas[1] * 57.2958f);
    // DEBUG_PRINT(" thetadot_meas3(deg/s):"); DEBUG_PRINT(local_thetadot_meas[2] * 57.2958f);
    // DEBUG_PRINT(" corrected_gyro1(deg/s):"); DEBUG_PRINT(local_corrected_gyro[0] * 57.2958f);
    // DEBUG_PRINT(" corrected_gyro2(deg/s):"); DEBUG_PRINT(local_corrected_gyro[1] * 57.2958f);
    // DEBUG_PRINT(" corrected_gyro3(deg/s):"); DEBUG_PRINT(local_corrected_gyro[2] * 57.2958f);

    // DEBUG_PRINT(" phidot_meas1(rad/s):"); DEBUG_PRINT(local_phidot_meas[0]);
    // DEBUG_PRINT(" phidot_meas2(rad/s):"); DEBUG_PRINT(local_phidot_meas[1]);
    // DEBUG_PRINT(" phidot_meas3(rad/s):"); DEBUG_PRINT(local_phidot_meas[2]);
    // DEBUG_PRINT(" phidot_filtered1(rad/s):"); DEBUG_PRINT(local_phidot_filtered[0]);
    // DEBUG_PRINT(" phidot_filtered2(rad/s):"); DEBUG_PRINT(local_phidot_filtered[1]);
    // DEBUG_PRINT(" phidot_filtered3(rad/s):"); DEBUG_PRINT(local_phidot_filtered[2]);
    // DEBUG_PRINT(" phidot_axis1(rad/s):"); DEBUG_PRINT(local_phidot_axis[0]);
    // DEBUG_PRINT(" phidot_axis2(rad/s):"); DEBUG_PRINT(local_phidot_axis[1]);
    // DEBUG_PRINT(" phidot_axis3(rad/s):"); DEBUG_PRINT(local_phidot_axis[2]);

    // DEBUG_PRINT(" Acce_sensor1(g):"); DEBUG_PRINT(local_acce_sensor[0]);
    // DEBUG_PRINT(" Acce_sensor2(g):"); DEBUG_PRINT(local_acce_sensor[1]);
    // DEBUG_PRINT(" Acce_sensor3(g):"); DEBUG_PRINT(local_acce_sensor[2]);
    // DEBUG_PRINT(" Acce_device1(g):"); DEBUG_PRINT(local_acce_device[0]);
    // DEBUG_PRINT(" Acce_device2(g):"); DEBUG_PRINT(local_acce_device[1]);
    // DEBUG_PRINT(" Acce_device3(g):"); DEBUG_PRINT(local_acce_device[2]);

    // DEBUG_PRINT(" sensor.getVelocity(deg/s):"); DEBUG_PRINT(sensor.getVelocity() * 57.2958f);
    // DEBUG_PRINT(" motor.getShaftVelocity(deg/s):"); DEBUG_PRINT(motor.getShaftVelocity() * 57.2958f);

    // DEBUG_PRINT(" theta_integral1:"); DEBUG_PRINT(local_theta_integral[0],4);
    // DEBUG_PRINT(" theta_integral2:"); DEBUG_PRINT(local_theta_integral[1],4);
    // DEBUG_PRINT(" theta_integral3:"); DEBUG_PRINT(local_theta_integral[2],4);

    // DEBUG_PRINT(" theta_error1:"); DEBUG_PRINT(local_theta_error[0] * 57.2958f);
    // DEBUG_PRINT(" theta_error2:"); DEBUG_PRINT(local_theta_error[1] * 57.2958f);
    // DEBUG_PRINT(" theta_error3:"); DEBUG_PRINT(local_theta_error[2] * 57.2958f);

    // DEBUG_PRINT(" phidot_integral1:"); DEBUG_PRINT(local_phidot_integral[0]);
    // DEBUG_PRINT(" phidot_integral2:"); DEBUG_PRINT(local_phidot_integral[1]);
    // DEBUG_PRINT(" phidot_integral3:"); DEBUG_PRINT(local_phidot_integral[2]);

    // DEBUG_PRINT(" Torque_axis1:"); DEBUG_PRINT(local_u_axis[0]);
    // DEBUG_PRINT(" Torque_axis2:"); DEBUG_PRINT(local_u_axis[1]);
    // DEBUG_PRINT(" Torque_axis3:"); DEBUG_PRINT(local_u_axis[2]);
    // DEBUG_PRINT(" Torque1:"); DEBUG_PRINT(local_u[0]);
    // DEBUG_PRINT(" Torque2:"); DEBUG_PRINT(local_u[1]);
    // DEBUG_PRINT(" Torque3:"); DEBUG_PRINT(local_u[2]);
    // DEBUG_PRINT(" Voltage1:"); DEBUG_PRINT(local_target_voltage[0]);
    // DEBUG_PRINT(" Voltage2:"); DEBUG_PRINT(local_target_voltage[1]);
    // DEBUG_PRINT(" Voltage3:"); DEBUG_PRINT(local_target_voltage[2]);
    // DEBUG_PRINT(" fbVoltage1:"); DEBUG_PRINT(local_feedback_voltage[0]);
    // DEBUG_PRINT(" fbVoltage2:"); DEBUG_PRINT(local_feedback_voltage[1]);
    // DEBUG_PRINT(" fbVoltage3:"); DEBUG_PRINT(local_feedback_voltage[2]);
    // DEBUG_PRINT(" ffVoltage1:"); DEBUG_PRINT(local_feedforward_voltage[0]);
    // DEBUG_PRINT(" ffVoltage2:"); DEBUG_PRINT(local_feedforward_voltage[1]);
    // DEBUG_PRINT(" ffVoltage3:"); DEBUG_PRINT(local_feedforward_voltage[2]);

    // Change printing pattern
    // DEBUG_PRINT(local_theta_error[0]); DEBUG_PRINT(",");
    // DEBUG_PRINT(local_theta_integral[0]); DEBUG_PRINT(",");
    // DEBUG_PRINT(local_u[0]);
    // DEBUG_PRINT(local_theta_error[1]); DEBUG_PRINT(",");
    // DEBUG_PRINT(local_theta_integral[1]); DEBUG_PRINT(",");
    // DEBUG_PRINT(local_u[1]);
    // DEBUG_PRINT(local_theta_error[2]); DEBUG_PRINT(",");
    // DEBUG_PRINT(local_theta_integral[2]); DEBUG_PRINT(",");
    // DEBUG_PRINT(local_u[2]);

    DEBUG_PRINTLN();
    
    // Delay to set the printing frequency (e.g., 20 Hz).
    vTaskDelay(pdMS_TO_TICKS(50));  // Run this task at ~20 Hz
  }
}
#endif



/*
// Timer initialization and ISR functions
// Timer interrupt service routine (ISR) - executes every period
// IRAM_ATTR makes sure the ISR is stored in instruction RAM (not flash), so it executes quickly and safely
void IRAM_ATTR onControlTimerWrapper(void* arg) {
  portENTER_CRITICAL_ISR(&timerMux);  // Start a critical section (disable interrupts to prevent race)
  controlUpdateFlag = true;  // Set a flag to signal the main loop that it's time to update control logic
  portEXIT_CRITICAL_ISR(&timerMux);  // Exit the critical section (re-enable interrupts)
}

// Initialize hardware timer
void initControlTimer() {
  const esp_timer_create_args_t periodic_timer_args = {
    .callback = &onControlTimerWrapper,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "control_timer"
  };

  esp_timer_create(&periodic_timer_args, &control_timer_handle);  // Create timer instance
  // esp_timer_start_periodic(control_timer_handle, 25000);  // each 25ms (40Hz)
  esp_timer_start_periodic(control_timer_handle, 20000);  // each 20ms (50Hz)
  // esp_timer_start_periodic(control_timer_handle, 15000);  // each 15ms (67Hz)  // Increase Control Loop Frequency

  Serial.println("Control timer initialized using esp_timer");
}
*/



// Serial command handlers
void doTargetVoltage(char* cmd) {
    // This command should ONLY work if we are already in serial mode.
    if (!serial_control_enabled) {
        Serial.println("Error: Must be in serial control mode. Press 'S' to toggle.");
        return;  // Exit the function
    }

    int motorIndex = 0;
    float voltage = 0;
    int argsParsed = sscanf(cmd, "%d %f", &motorIndex, &voltage);  // Parse command like "T 1 2.5"

    if (argsParsed == 2 && motorIndex >= 0 && motorIndex < 3) {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
          sharedState.target_voltage[motorIndex] = Limit(voltage, -max_voltage, max_voltage);  // Clamp voltage
          xSemaphoreGive(stateMutex);
        }
        Serial.print("Motor "); Serial.print(motorIndex);
        Serial.print(" voltage set to: "); Serial.println(voltage);

    } else if (argsParsed == 2 && motorIndex == 3) {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
          sharedState.target_voltage[0] = Limit(voltage, -max_voltage, max_voltage);  // Clamp voltage
          sharedState.target_voltage[1] = Limit(voltage, -max_voltage, max_voltage);  // Clamp voltage
          sharedState.target_voltage[2] = Limit(voltage, -max_voltage, max_voltage);  // Clamp voltage
          xSemaphoreGive(stateMutex);
        }
        Serial.print("All motors' voltage set to: "); Serial.println(voltage);
    
    } else if (argsParsed == 1 && motorIndex == 10) {  // reset to zero
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
          sharedState.target_voltage[0] =  0.0f;
          sharedState.target_voltage[1] =  0.0f;
          sharedState.target_voltage[2] =  0.0f;
          xSemaphoreGive(stateMutex);
        }
        Serial.println("reset to zero");
    
    } else if (argsParsed == 1 && motorIndex == 11) {  // axis-x test
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
          sharedState.target_voltage[0] =  6.667f;
          sharedState.target_voltage[1] = -3.333f;
          sharedState.target_voltage[2] = -3.333f;
          xSemaphoreGive(stateMutex);
        }
        Serial.println("axis-x test");

    } else if (argsParsed == 1 && motorIndex == 12) {  // axis-y test
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
          sharedState.target_voltage[0] =  0.000f;
          sharedState.target_voltage[1] =  5.774f;
          sharedState.target_voltage[2] = -5.774f;
          xSemaphoreGive(stateMutex);
        }
        Serial.println("axis-y test");
    
    } else if (argsParsed == 1 && motorIndex == 13) {  // axis-z test
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
          sharedState.target_voltage[0] = -3.333f;
          sharedState.target_voltage[1] = -3.333f;
          sharedState.target_voltage[2] = -3.333f;
          xSemaphoreGive(stateMutex);
        }
        Serial.println("axis-z test");

    } else {
        Serial.println("Usage: T [0-2,3(all)] [voltage]");
    }
}


void doToggleSerial(char* cmd) {
  serial_control_enabled = !serial_control_enabled;  // Flip the mode (toggle)

  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    if (serial_control_enabled) {
      Serial.println("Serial control ENABLED. LQR is paused.");
      for (int i = 0; i < 3; i++) {
        sharedState.target_voltage[i] = 0;  // Reset voltages when enabling serial control
        sharedState.theta_integral[i] = 0;  // Reset integrals to prevent windup
        sharedState.phidot_integral[i] = 0;  // Reset integrals to prevent windup
      }
    } else {
      Serial.println("Serial control DISABLED. Resuming LQR balance mode.");
    }
    xSemaphoreGive(stateMutex);
  }
}


/*
void doDebugSensors(char* cmd) {
    for (int i = 0; i < 3; i++) {
        sensors[i].selectMuxChannel();
        float angle = sensors[i].getAngle();
        Serial.print("Sensor "); Serial.print(i); 
        Serial.print(" angle: "); Serial.println(angle);
    }
}
*/


// Serial command handler for averaging Acce_sensor
void doAcceAverage(char* cmd) {
  Serial.println("Averaging Acce_sensor for 10 seconds...");
  
  float avg_acce[3] = {0.0f, 0.0f, 0.0f};
  float local_acce[3];
  // EstimationTask runs every 10ms, so 100 samples = 1 second.
  const int num_samples = 1000; 

  for (int i = 0; i < num_samples; i++) {
    // Safely get the latest accelerometer data from sharedState
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
      // Read from acce_sensor instead of acce_device
      memcpy(local_acce, sharedState.acce_sensor, sizeof(local_acce));
      xSemaphoreGive(stateMutex);
    }
    
    // Accumulate the readings
    avg_acce[0] += local_acce[0];
    avg_acce[1] += local_acce[1];
    avg_acce[2] += local_acce[2];
    
    // Delay for 10ms. This loop runs in the low-priority loop() task
    // to sample at approximately the 100Hz rate of the EstimationTask.
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
  
  // Calculate the average
  avg_acce[0] /= num_samples;
  avg_acce[1] /= num_samples;
  avg_acce[2] /= num_samples;
  
  // Print the final result
  Serial.print("Average Acce_sensor (10s): [");
  Serial.print(avg_acce[0], 5);
  Serial.print(", ");
  Serial.print(avg_acce[1], 5);
  Serial.print(", ");
  Serial.print(avg_acce[2], 5);
  Serial.println("] (g)");
}


void doSetVertexK(char* cmd) {
    int row, col;
    float val;
    int argsParsed = sscanf(cmd, "%d %d %f", &row, &col, &val);
    if (argsParsed == 3 && row == 0) {
        mode_manag.setVertexGain(0, col-1, val);
        mode_manag.setVertexGain(1, col-1, val);
        mode_manag.setVertexGain(2, col-1, val);
        Serial.print("Set K_vertex["); Serial.print("all]["); 
        Serial.print(col); Serial.print("] = "); Serial.println(val, 5);
    } else if (argsParsed == 3) {
        mode_manag.setVertexGain(row-1, col-1, val);
        Serial.print("Set K_vertex["); Serial.print(row); Serial.print("][");
        Serial.print(col); Serial.print("] = "); Serial.println(val, 5);
    } else {
        Serial.println("Err: K [row 0-3] [col 1-3] [val]");
    }
}


void doIncVertexK(char* cmd) {
    // Skip potential leading spaces
    while (*cmd == ' ') cmd++;

    int row = cmd[0] - '0';
    int col = cmd[1] - '0';
    char op = cmd[2];

    // Validate input: row/col must be 1-3, op must be + or -
    if (row < 1 || row > 3 || col < 1 || col > 3 || (op != '+' && op != '-')) {
        Serial.println("Err: k [row][col][+/-] e.g. 11+");
        return;
    }

    // Get current value
    float current_val = mode_manag.getVertexGain(row - 1, col - 1);
    float increment = 0;

    // Determine increment unit based on column (State variable)
    if (col == 1) increment = 1.0f;  // Col 1 (theta)
    else if (col == 2) increment = 0.1f;  // Col 2 (theta_dot)
    else if (col == 3) increment = 0.001f;  // Col 3 (phi_dot)

    // Apply sign
    if (op == '-') increment = -increment;

    float new_val = current_val + increment;
    
    // Update gain
    mode_manag.setVertexGain(row - 1, col - 1, new_val);
    
    // Show new value
    Serial.print("Set K_vertex["); Serial.print(row); Serial.print("][");
    Serial.print(col); Serial.print("] = "); Serial.println(new_val, 5);
}


void doSetVertexKi(char* cmd) {
    int axis;
    float val;
    int argsParsed = sscanf(cmd, "%d %f", &axis, &val);
    if (argsParsed == 2 && axis == 0) {
        mode_manag.setVertexKi(0, val);
        mode_manag.setVertexKi(1, val);
        mode_manag.setVertexKi(2, val);
        Serial.print("Set Ki_vertex[all");
        Serial.print("] = "); Serial.println(val, 5);
    } else if (argsParsed == 2) {
        mode_manag.setVertexKi(axis-1, val);
        Serial.print("Set Ki_vertex["); Serial.print(axis);
        Serial.print("] = "); Serial.println(val, 5);
    } else {
        Serial.println("Err: I [axis 0-3] [val]");
    }
}


void doSetVertexQu(char* cmd) {
    float q0, q1, q2, q3;
    int argsParsed = sscanf(cmd, "%f %f %f %f", &q0, &q1, &q2, &q3);
    if (argsParsed == 4) {
        mode_manag.setVertexQuat(q0, q1, q2, q3);
        Serial.print("Set qu_VERTEX: "); 
        Serial.print(q0, 4); Serial.print(", "); Serial.print(q1, 4); Serial.print(", ");
        Serial.print(q2, 4); Serial.print(", "); Serial.println(q3, 4);
    } else {
        Serial.println("Err: Q [q0] [q1] [q2] [q3]");
    }
}


// Serial command to control spin
void doSpin(char* cmd) {
    float vel;
    int argsParsed = sscanf(cmd, "%f", &vel);
    if (argsParsed == 1) {
        // Limit maximum speed for safety (e.g., 2.0 rad/s)
        if(vel > 2.0f) vel = 2.0f;
        if(vel < -2.0f) vel = -2.0f;
        
        att_traj.setTargetVelocity(vel);
        Serial.print("Spin velocity set to: "); Serial.println(vel);
    } else {
        Serial.println("Usage: V [velocity rad/s]");
    }
}