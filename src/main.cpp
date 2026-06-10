#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cmath>

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

    for (int t = 0; t < 200; ++t) {
        double current_time = t * dt;

        // SCENARIO at t = 5s: lead vehicle stops and target gap changes
        if (current_time >= 5.0) {
            lead_velocity = 0.0;
            setpoint = 25.0;
        }

        // 1. Gap error
        double gap_error = distance - setpoint;

        // 2. Relative velocity
        double relative_velocity = velocity - lead_velocity;

        // 3. TTC
        double ttc;
        if (relative_velocity > closing_deadband) {
            ttc = distance / relative_velocity;
        } else {
            ttc = std::numeric_limits<double>::infinity();
        }

        // 4. PD control
        double output = (kp_gap * gap_error) - (kv * relative_velocity);

        // 5. Emergency brake — fires when TTC critical AND closing fast enough
        if (ttc < ttc_threshold) {
            output = max_brake;  // Full -8 m/s²
        }

        // 6. Clamp
        double acceleration = std::clamp(output, max_brake, max_accel);

        // 8. Log — cap TTC at 20s for clean plotting, -1 for infinity
        double ttc_log;
        if (std::isinf(ttc)) {
            ttc_log = -1.0;
        } else if (ttc > 20.0) {
            ttc_log = -1.0;  // off-chart, treat as N/A
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

        // 9. Integrate
        velocity += acceleration * dt;
        if (velocity < 0.0) velocity = 0.0;

        distance += (lead_velocity - velocity) * dt;
        if (distance < 0.0) distance = 0.0;
    }

    file.close();
    std::cout << "Simulation complete. data.csv updated." << std::endl;
    return 0;
}