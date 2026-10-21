#include "pid_controller.hpp"

PIDController::PIDController(double kp, double ki, double kd, double max_output) 
  : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output),
    integral_(0.0), prev_error_(0.0), first_call_(true) {}

double PIDController::update(double error, double dt) {
  if (dt <= 0.0) return 0.0;
  integral_ += error * dt;
  if (ki_ > 1e-6) {
    double integral_max = max_output_ / ki_;
    if (integral_ > integral_max) integral_ = integral_max;
    if (integral_ < -integral_max) integral_ = -integral_max;
  }
  double derivative = 0.0;
  if (first_call_) first_call_ = false;
  else derivative = (error - prev_error_) / dt;
  prev_error_ = error;
  double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  if(output > max_output_) output = max_output_;
  if(output < -max_output_) output = -max_output_;
  return output;
}
    
void PIDController::reset() {
  integral_ = 0.0;
  prev_error_ = 0.0;
  first_call_ = true;
}
