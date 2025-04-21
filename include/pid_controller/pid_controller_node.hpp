#ifndef PID_CONTROLLER_NODE_HPP
#define PID_CONTROLLER_NODE_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "pid_controller/pid.hpp"

namespace pid_controller {

class PIDControllerNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the PID controller ROS node
     * 
     * @param node_options Additional options for the node
     */
    explicit PIDControllerNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

    /**
     * @brief Destructor
     */
    ~PIDControllerNode() = default;

private:
    // PID controller instance
    std::unique_ptr<PID> pid_controller_;
    
    // ROS2 publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setpoint_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pid_enable_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Callback for parameter changes
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    
    // Internal state
    double current_state_{0.0};
    double current_setpoint_{0.0};
    bool has_received_state_{false};
    bool has_received_setpoint_{false};
    bool auto_mode_{true};
    bool reverse_action_{false};
    
    // Callback functions
    void state_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void setpoint_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void pid_enable_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void control_loop();
    
    // Parameter handling
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> & parameters);
    
    // Initialize the PID controller with current parameters
    void init_controller();
};

} // namespace pid_controller

#endif // PID_CONTROLLER_NODE_HPP 