#include "pid_controller.hpp"

PIDController::PIDController(double kp, double ki, double kd, double max_output) 
  : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output),
    integral_(0.0), prev_error_(0.0), first_call_(true) {}

double PIDController::update(double error, double dt) {
  if (dt <= 0.0) return 0.0;
  integral_ += error * dt;    // Интеграл с anti-windup
  if (ki_ > 1e-6) {
    double integral_max = max_output_ / ki_;
    if (integral_ > integral_max) integral_ = integral_max;
    if (integral_ < -integral_max) integral_ = -integral_max;
  } else integral_ = 0.0;
  // Производная с фильтром низких частот
  double derivative = 0.0;
  if (first_call_) {
    first_call_ = false;
    prev_derivative_ = 0.0; // Initialize filtered derivative
  } else {
    double raw_derivative = (error - prev_error_) / dt;
    // Экспоненциальное сглаживание (EMA)
    prev_derivative_ = alpha_ * raw_derivative + (1.0 - alpha_) * prev_derivative_;
    derivative = prev_derivative_;
  }
  prev_error_ = error;
  // Выход PID
  double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  if(output > max_output_) output = max_output_;
  if(output < -max_output_) output = -max_output_;
  return output;
}
    
void PIDController::reset() {
  integral_ = 0.0;
  prev_error_ = 0.0;
  prev_derivative_ = 0.0;
  first_call_ = true;
}
   
void PIDController::set_gains(double kp, double ki, double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::set_max_output(double max_output) {
  max_output_ = max_output;
}

void PIDController::set_filter_alpha(double alpha) {
  // Ограничиваем значение от 0.0 до 1.0
  if (alpha < 0.0) alpha = 0.0;
  if (alpha > 1.0) alpha = 1.0;
  alpha_ = alpha;
}