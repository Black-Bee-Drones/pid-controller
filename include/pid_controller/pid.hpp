#ifndef PID_CONTROLLER_PID_HPP
#define PID_CONTROLLER_PID_HPP

#include <chrono>
#include <limits>
#include <cmath>

namespace pid_controller {

class PID {
public:
    /**
     * @brief Constructor for PID controller
     * 
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     * @param setpoint Target setpoint
     * @param output_limits Range for output clamping {min, max}
     * @param integral_limits Range for integral term clamping {min, max}
     */
    PID(
        double p = 0.0,
        double i = 0.0,
        double d = 0.0,
        double setpoint = 0.0,
        std::pair<double, double> output_limits = {-std::numeric_limits<double>::infinity(), 
                                                std::numeric_limits<double>::infinity()},
        std::pair<double, double> integral_limits = {-std::numeric_limits<double>::infinity(), 
                                                 std::numeric_limits<double>::infinity()}
    );

    /**
     * @brief Calculate PID output value for given reference input and feedback
     * 
     * @param current_value Current value of the process variable
     * @return double Control output
     */
    double update(double current_value);

    /**
     * @brief Reset the PID controller's integral term and timing
     */
    void reset();

    /**
     * @brief Update the setpoint
     * 
     * @param new_setpoint New target setpoint
     */
    void set_setpoint(double new_setpoint);

    /**
     * @brief Update the PID gains
     * 
     * @param p New proportional gain
     * @param i New integral gain
     * @param d New derivative gain
     */
    void tune(double p, double i, double d);

    // Direct access to gains for dynamic reconfiguration
    double kp;
    double ki;
    double kd;
    double setpoint;
    std::pair<double, double> output_limits;
    std::pair<double, double> integral_limits;
    double output; // Last calculated output

private:
    // PID components
    double proportional_{0.0};
    double integral_{0.0};
    double derivative_{0.0};
    
    // State tracking
    std::chrono::time_point<std::chrono::steady_clock> last_time_;
    double last_error_{0.0};
    bool first_update_{true};
};

} // namespace pid_controller

#endif // PID_CONTROLLER_PID_HPP 