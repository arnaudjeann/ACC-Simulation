# Adaptive Cruise Control (ACC) Simulation with PD Control

A C++ and Python simulation of an Adaptive Cruise Control (ACC) system. This project implements a Proportional-Derivative (PD) control loop to regulate a vehicle's trailing distance and velocity based on physical constraints.

## Overview

The simulation models an "Ego" vehicle tracking a "Lead" vehicle moving at a constant speed of 10 m/s. The ego vehicle adapts its acceleration and braking to maintain a target distance, including during a step-change scenario where the target distance shortens.

### Key Features
* **PD Control Loop:** Implemented in C++ to calculate control outputs.
* **Actuator Clamping:** Uses physical bounds modeled after modern electric vehicles (Max acceleration: $+3.0\text{ m/s}^2$, Max braking: $-8.0\text{ m/s}^2$).
* **Derivative via Relative Velocity:** Uses the physical delta ($v_{lead} - v_{ego}$) for derivative damping, avoiding numerical differentiation and integral wind-up.
* **Visualization:** Exports simulation logs into a `data.csv` file, processed and rendered using Python (`pandas` and `matplotlib`).

---

## Simulation Results

The controller handles a sudden setpoint change at $t = 5\text{ s}$, shifting the target distance from 40m down to 25m:

1. **Distance Tracking:** Converges to the target thresholds without overshoot or oscillations.
2. **Velocity Profile:** Matches the lead vehicle's speed ($10\text{ m/s}$) at steady state, with a temporary acceleration phase to close the gap when the setpoint changes.
3. **Actuator Output:** Operates within the defined physical limits of the clamping logic.

![Simulation Telemetry](<telemetry_test.png>)

---

## Project Structure

* `src/main.cpp`: The C++ simulation loop and PD controller logic.
* `scripts/plot_results.py`: Python script utilizing Matplotlib to generate telemetry plots.
* `data.csv`: Output telemetry containing time, distance, velocity, and acceleration logs.

---

## How to Run

### Prerequisites
* A C++17 compatible compiler (e.g., Clang, GCC)
* CMake
* Python 3.x with `pandas` and `matplotlib` installed

### Step 1: Compile and Run the C++ Simulation
Generate the build system and run the binary to output the simulation data:
```bash
# Configure and build
cmake -B out/build
cmake --build out/build

# Run the simulation
./out/build/acc_sim