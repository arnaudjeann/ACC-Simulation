#include <iostream>
#include <fstream>
#include <algorithm>

int main() {
    std::ofstream file("data.csv");
    file << "time,distance,velocity,acceleration\n";

    // INITIAL STATE 
    double distance = 60.0;      // Starting distance from lead vehicle (m)
    double velocity = 15.0;      // Ego vehicle velocity (m/s)
    double lead_velocity = 10.0; // Lead vehicle velocity (m/s)
    double dt = 0.1;             // Time step (100ms)
    
    // PD CONTROLLER PARAMETERS
    double setpoint = 40.0; 
    
    // Controller gains:
    double kp = 0.2;  // Proportional gain: pushes toward the target distance
    double kd = 0.8;  // Derivative gain: dampens the approach if closing in too fast

    double max_accel = 3.0; 
    double max_brake = -8.0; 

    // SIMULATION LOOP
    for (int t = 0; t < 200; ++t) {
        double current_time = t * dt;

        // SCENARIO: Lead vehicle suddenly brakes at 5 seconds (target distance shortens)
        if (current_time > 5.0) setpoint = 25.0;

        // 1. Distance error (Positive = too far, need to accelerate)
        double error = distance - setpoint; 
        
        // 2. The Derivative component IS the relative velocity
        // If lead vehicle is at 10 m/s and ego is at 15 m/s, derivative is -5 m/s (distance closing)
        double derivative = lead_velocity - velocity; 
        
        // 3. Control law calculation (PD control output)
        double output = (kp * error) + (kd * derivative);

        // 4. Actuator clamping: Apply physical constraints
        double acceleration = std::clamp(output, max_brake, max_accel);

        // 5. Vehicle dynamics update
        velocity += acceleration * dt;
        if (velocity < 0) velocity = 0; // Prevent reversing
        
        // Track relative distance changes based on velocity delta between both vehicles
        distance += (lead_velocity - velocity) * dt;

        // Log data points
        file << current_time << "," << distance << "," << velocity << "," << acceleration << "\n";
    }

    file.close();
    std::cout << "PD Simulation completed successfully. data.csv updated." << std::endl;
    return 0;
}