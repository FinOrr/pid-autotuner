#include <iostream>
#include "pid_controller.hpp"

int main() {
    // Create a PID object
    PIDController pid(1.0, 0.1, 0.01);

    // Simulation parameters
    double setpoint = 10.0;
    double initial_process_variable = 5.0;
    double time_step = 0.1;

    // Simulate the PID controller in action
    for (int i = 0; i < 20; ++i) {
        double pid_output = pid.calculate(setpoint, initial_process_variable, time_step);
        std::cout << "Iteration " << i + 1 << ": PID output: " << pid_output << std::endl;
        // Update the process variable for the next iteration
        initial_process_variable += pid_output;
    }

    return 0;
}
