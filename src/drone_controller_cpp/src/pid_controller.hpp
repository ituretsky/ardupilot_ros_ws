#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double max_output);

    double update(double error, double dt);
    void reset();
    void set_gains(double kp, double ki, double kd);
    void set_max_output(double max_output);

private:
    double kp_;
    double ki_;
    double kd_;
    double max_output_;

    double integral_ = 0.0;
    double prev_error_ = 0.0;
    bool first_call_ = true;
};

#endif  // PID_CONTROLLER_HPP
