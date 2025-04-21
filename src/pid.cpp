#include "pid_controller/pid.hpp"

namespace pid_controller {

PID::PID(
    double p,
    double i,
    double d,
    double setpoint_val,
    std::pair<double, double> output_limits_val,
    std::pair<double, double> integral_limits_val
) : kp(p),
    ki(i),
    kd(d),
    setpoint(setpoint_val),
    output_limits(output_limits_val),
    integral_limits(integral_limits_val),
    output(0.0),
    last_time_(std::chrono::steady_clock::now())
{
}

double PID::update(double current_value) {
    // Get current time
    auto current_time = std::chrono::steady_clock::now();
    
    // Calculate time delta
    std::chrono::duration<double> time_delta = current_time - last_time_;
    double dt = time_delta.count();
    
    // Avoid division by zero or huge derivative on first run or very small dt
    if (first_update_ || dt < 1e-6) {
        first_update_ = false;
        last_time_ = current_time;
        return output; // Return last computed output
    }
    
    // Calculate error
    double error = setpoint - current_value;
    
    // Proportional term
    proportional_ = kp * error;
    
    // Integral term (with anti-windup)
    integral_ += ki * error * dt;
    
    // Apply integral limits (anti-windup)
    integral_ = std::max(std::min(integral_, integral_limits.second), integral_limits.first);
    
    // Derivative term
    double error_diff = error - last_error_;
    derivative_ = kd * error_diff / dt;
    
    // Calculate total output
    output = proportional_ + integral_ + derivative_;
    
    // Apply output limits
    output = std::max(std::min(output, output_limits.second), output_limits.first);
    
    // Store state for next iteration
    last_error_ = error;
    last_time_ = current_time;
    
    return output;
}

void PID::reset() {
    integral_ = 0.0;
    last_error_ = 0.0;
    first_update_ = true;
    last_time_ = std::chrono::steady_clock::now();
    output = 0.0;
}

void PID::set_setpoint(double new_setpoint) {
    setpoint = new_setpoint;
}

void PID::tune(double p, double i, double d) {
    kp = p;
    ki = i;
    kd = d;
}

} // namespace pid_controller 