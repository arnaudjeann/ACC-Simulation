# Adaptive Cruise Control (ACC) Simulation with PD Control and TTC

A C++ and Python simulation of an Adaptive Cruise Control (ACC) system. The project uses a Proportional-Derivative (PD) control loop to regulate the distance between a controlled vehicle and a lead vehicle, while also computing a dynamic Time-to-Collision (TTC).

## Overview

The simulation starts with two vehicles traveling at `10 m/s` with a stable following distance of `40 m`. At `t = 5 s`, the lead vehicle stops and the target following distance changes from `40 m` to `25 m`.

The controlled vehicle reacts by braking smoothly, respecting acceleration and braking limits. The simulation also calculates TTC to detect whether an emergency braking condition should be triggered.

### Key Features

* **PD Control Loop:** Calculates acceleration from distance error and relative velocity.
* **Relative Velocity Damping:** Uses `v_ego - v_lead` to reduce closing speed and avoid oscillations.
* **Actuator Clamping:** Limits acceleration to `+3.0 m/s^2` and braking to `-8.0 m/s^2`.
* **Time-to-Collision (TTC):** Computes TTC when the controlled vehicle is closing in on the lead vehicle.
* **Emergency Braking Logic:** Applies maximum braking if TTC drops below the critical threshold of `2.0 s`.
* **Visualization:** Exports telemetry to `data.csv`, then plots it with Python using `pandas` and `matplotlib`.

---

## Simulation Scenario

Initial conditions:

* Distance: `40 m`
* Controlled vehicle velocity: `10 m/s`
* Lead vehicle velocity: `10 m/s`
* Target distance: `40 m`

At `t = 5 s`:

* Lead vehicle velocity becomes `0 m/s`
* Target distance becomes `25 m`

The controller then slows the controlled vehicle until it stops near the new target distance.

---

## Simulation Results

The generated plot contains four graphs:

1. **Distance Tracking:** Shows the actual following distance converging from `40 m` toward the new `25 m` target.
2. **Velocity Profile:** Shows the controlled vehicle slowing down after the lead vehicle stops.
3. **Acceleration:** Shows the braking command staying within the physical limits.
4. **Time-to-Collision:** Shows TTC staying above the critical `2.0 s` threshold in the current scenario.

In this scenario, emergency braking does not trigger because TTC remains above the critical threshold.

![Simulation Telemetry](<telemetry_test.png>)

---

## Project Structure

* `src/main.cpp`: C++ simulation loop, PD controller, TTC calculation, and CSV export.
* `scripts/plot_results.py`: Python script that plots distance, velocity, acceleration, and TTC.
* `data.csv`: Generated telemetry file containing `time`, `distance`, `velocity`, `lead_velocity`, `acceleration`, `setpoint`, and `ttc`.

---

## How to Run

### Prerequisites

* A C++17 compatible compiler, such as Clang or GCC
* CMake
* Python 3.x
* Python packages: `pandas`, `matplotlib`, and `numpy`

### Step 1: Compile and Run the C++ Simulation

```bash
cmake -B out/build
cmake --build out/build
./out/build/acc_sim
```

This generates or updates `data.csv`.

### Step 2: Plot the Results

```bash
python3 scripts/plot_results.py
```

This opens the Matplotlib window with the four telemetry graphs.
