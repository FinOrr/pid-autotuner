#include "pid_controller.hpp"

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      integral_(0),
      prev_error_(0),
      min_output_(std::numeric_limits<double>::lowest()),
      max_output_(std::numeric_limits<double>::max()) {}

double PIDController::calculate(double setpoint, double process_variable,
                                double dt) {
  // The error is the difference between the setpoint and the process variable
  double error = setpoint - process_variable;
  // The integral is the sum of the error over time
  integral_ += error * dt;
  // The derivative is the rate of change of the error
  double derivative = (error - prev_error_) / dt;
  // The output is the sum of the proportional, integral, and derivative
  double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

  // If the output is less than the minimum output, we need to increase it to
  // be within the limits and prevent the system from overshooting the setpoint
  if (output < min_output_) {
    output = min_output_;
    if (error > 0) {
      integral_ -= error * dt;
    }
  } else if (output > max_output_) {
    output = max_output_;
    if (error < 0) {
      integral_ -= error * dt;
    }
  }

  prev_error_ = error;

  return (output);
}

void PIDController::setOutputLimits(double min, double max) {
  min_output_ = min;
  max_output_ = max;
}

void PIDController::reset() {
  integral_ = 0;
  prev_error_ = 0;
}

void PIDController::setCoefficients(double kp, double ki, double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}
