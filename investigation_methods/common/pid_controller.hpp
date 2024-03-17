#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP
#include <limits>

class PIDController {
 public:
  // Constructor for the PID controller
  PIDController(double kp, double ki, double kd);
  // Calculate the output of the PID controller
  double calculate(double setpoint, double process_variable, double dt);
  // Set the output limits of the PID controller
  void setOutputLimits(double min, double max);
  // Set the coefficients of the PID controller
  void setCoefficients(double kp, double ki, double kd);
  // Reset the integral and previous error
  void reset();

 private:
  double kp_;
  double ki_;
  double kd_;
  double integral_;
  double prev_error_;
  double min_output_;
  double max_output_;
};

#endif  // PID_CONTROLLER_HPP
