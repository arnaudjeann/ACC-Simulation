#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cmath>
#include <queue>
#include <random>

int main() {
    std::ofstream file("data.csv");
    file << "time,distance,velocity,lead_velocity,acceleration,setpoint,ttc\n";

    // INITIAL STATE
    double distance = 40.0;
    double velocity = 10.0;
    double lead_velocity = 10.0;
    double dt = 0.1;

    // CONTROLLER PARAMETERS
    double setpoint = 40.0;
    double max_accel =  3.0;
    double max_brake = -8.0;
    double kp_gap = 0.15;
    double kv = 0.8;

    // TTC
    double ttc_threshold = 2.0;
    double closing_deadband = 0.5; // (m/s)

    double prev_acceleration = 0.0;
    double max_jerk = 15.0;
    double actuator_delay = 0.3;

    int delay_steps = static_cast<int>(actuator_delay / dt);
    std::queue<double> accel_command_queue;
    for (int i = 0; i < delay_steps; ++i) {
        accel_command_queue.push(0.0); // Initialize queue with zero acceleration
    }

    std::default_random_engine generator(42); // Fixed seed for reproductibility
    std::normal_distribution<double> dist_noise(0.0, 0.4); // Mean 0, Std Dev 0.4 meters
    std::normal_distribution<double> vel_noise(0.0, 0.1); // Mean 0, Std Dev 0.1 m/s

    for (int t = 0; t < 200; ++t) {
        double current_time = t * dt;

        // SCENARIO at t = 5s: lead vehicle stops and target gap changes
        if (current_time >= 5.0) {
            lead_velocity = 0.0;
            setpoint = 25.0;
        }

        // 1. Add Environmental Uncertainty (Sensor Noise)
        // The car acts on "measured" values, not perfectly clean ground truths
        double measured_distance = distance + dist_noise(generator);
        double measured_ego_velocity = velocity + vel_noise(generator);
        double measured_relative_velocity = measured_ego_velocity - lead_velocity;

        // 2. Gap error
        double gap_error = distance - setpoint;


        // 3. TTC
        double ttc;
        if (measured_relative_velocity > closing_deadband) {
            ttc = distance / measured_relative_velocity;
        } else {
            ttc = std::numeric_limits<double>::infinity();
        }

        // 4. PD control
        double output = (kp_gap * gap_error) - (kv * measured_relative_velocity);

        // 5. Emergency brake — fires when TTC critical AND closing fast enough
        if (ttc < ttc_threshold) {
            output = max_brake;  // Full -8 m/s²
        }

        // 6. Clamp
        double raw_acceleration_command = std::clamp(output, max_brake, max_accel);

        // 7. Actuator Delay (Latency Queue)
        accel_command_queue.push(raw_acceleration_command);
        double delayed_command = accel_command_queue.front();
        accel_command_queue.pop();

        // 8. Jerk Limiting
        double acceleration = std::clamp(delayed_command, 
            prev_acceleration - max_jerk * dt, 
            prev_acceleration + max_jerk * dt);

        prev_acceleration = acceleration;

        // 9. Log — cap TTC at 20s for clean plotting, -1 for infinity
        double ttc_log;
        if (std::isinf(ttc) || ttc > 20.0) {
            ttc_log = -1.0;
        } else {
            ttc_log = ttc;
        }

        file << current_time << ","
             << distance << ","
             << velocity << ","
             << lead_velocity << ","
             << acceleration << ","
             << setpoint << ","
             << ttc_log << "\n";

        // 10. Physics Integration
        velocity += acceleration * dt;
        if (velocity < 0.0) velocity = 0.0;

        distance += (lead_velocity - velocity) * dt;
        if (distance < 0.0) distance = 0.0;
    }

    file.close();
    std::cout << "Simulation complete. data.csv updated." << std::endl;
    return 0;
}