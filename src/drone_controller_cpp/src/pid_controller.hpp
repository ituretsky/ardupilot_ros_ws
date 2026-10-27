#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double max_output);

    double update(double error, double dt);
    void reset();
    void set_gains(double kp, double ki, double kd);
    void set_max_output(double max_output);
    void set_filter_alpha(double alpha);

private:
    double kp_;                    // пропорциональный коэффициент
    double ki_;                    // интегральный (инерция движения)
    double kd_;                    // дифференциальный (ускорение)
    double max_output_;            // ограничение скорости
    double integral_ = 0.0;        // накопление ошибки
    double prev_error_ = 0.0;      // предыдущая ошибка (расстояние до цели)
    bool   first_call_ = true;     // флаг первого вызова
    double alpha_ = 0.1;           // коэффициент фильтра (от 0 до 1)
    double prev_derivative_ = 0.0; // сглаженная производная с прошлого шага
};

#endif  // PID_CONTROLLER_HPP
