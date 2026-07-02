# Self-Stabilizing Cube

### Integrating Dynamics & Optimal Control for Mechatronic Systems

The Self-Stabilizing Cube is an open-source testbed designed to address the fundamental challenge of stabilizing an open-loop unstable system using advanced feedback control. Inspired by real-world aerospace attitude control systems, the cube is capable of balancing itself on one of its corners and simultaneously rotating around its axis in a controlled manner.

**Full Documentation & Visuals:** [Project Portfolio](https://imjuliuschen.github.io/#cube)

---

## Project Overview

As a complex mechatronic device, the system utilizes three internal reaction wheels mounted orthogonally to each face of the cube for precise actuation. This repository contains the complete C++ firmware required to run the stabilization and estimation loops. 

As the sole designer, builder, and tuner of this project, I systematically integrated 3D mechanical design, embedded systems, and optimal control theory to achieve rapid and robust stabilization. 

### Key Technical Achievements

* **Advanced Control Theory:** The system dynamics were derived using Lagrange equations. To address the highly unstable, nonlinear dynamics, the equations of motion were linearized about the unstable equilibrium point (pendulum up) to apply the LQR optimal control framework.

* **Decaying Integrator:** A major challenge was the initial control instability caused by physical imperfections (uneven weight distribution and installation errors). This equilibrium mismatch was solved by augmenting the LQR control law with a gradually decaying integral term, allowing the controller to actively track the true physical equilibrium.

* **Quaternion-Based Estimation:** Robust, singularity-free attitude estimation was achieved utilizing a quaternion-based complementary filter (AHRS), transforming noisy accelerometer and drifting gyroscope data from a single MPU-6050 IMU into actionable control data.

* **Optimized Real-Time Embedded Systems:** The firmware is built in C++ leveraging FreeRTOS for multi-core task scheduling. The fast-running motor FOC task is assigned to Core 0, while the critical 100Hz Estimation and Control tasks run on Core 1, eliminating latency and phase lag. 

* **Hardware Execution:** The prototype utilizes a custom-soldered prototype board with internal wiring to interface the ESP32, BLDC motors, and I2C sensors.

### Performance 

Through rigorous model validation and invariant analysis, the physical prototype successfully recovers from a 10° inclination disturbance and settles in less than 1 second.

## Repository Structure

* `/Code`: Contains all C++ header (`.h`) and source (`.cpp`) files, alongside the main sketch (`.ino`), covering attitude estimation, the LQR controller, motor drivers, and mode management.
