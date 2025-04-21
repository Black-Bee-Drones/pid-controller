#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class PIDTestEnvironment : public rclcpp::Node
{
public:
  PIDTestEnvironment() : Node("pid_test_environment"), time_(0.0), setpoint_(5.0), state_(0.0)
  {
    // Parameters
    this->declare_parameter("state_topic", "/test/state");
    this->declare_parameter("setpoint_topic", "/test/setpoint");
    this->declare_parameter("control_effort_topic", "/test/control_effort");
    this->declare_parameter("publish_frequency", 20.0);  // Hz
    this->declare_parameter("noise_level", 0.1);         // Noise amplitude for state
    this->declare_parameter("simulation_rate", 0.2);     // How fast the system responds

    // Get parameters
    std::string state_topic = this->get_parameter("state_topic").as_string();
    std::string setpoint_topic = this->get_parameter("setpoint_topic").as_string();
    std::string control_effort_topic = this->get_parameter("control_effort_topic").as_string();
    double publish_frequency = this->get_parameter("publish_frequency").as_double();

    // Create publishers and subscribers
    state_publisher_ = this->create_publisher<std_msgs::msg::Float64>(state_topic, 10);
    setpoint_publisher_ = this->create_publisher<std_msgs::msg::Float64>(setpoint_topic, 10);

    control_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        control_effort_topic, 10, std::bind(&PIDTestEnvironment::control_effort_callback, this, std::placeholders::_1));

    // Create timer for simulation updates
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / publish_frequency)),
                                     std::bind(&PIDTestEnvironment::timer_callback, this));

    // Create setpoint change timer - changes every 10 seconds
    setpoint_timer_ = this->create_wall_timer(10s, std::bind(&PIDTestEnvironment::setpoint_change_callback, this));

    RCLCPP_INFO(this->get_logger(), "PID Test Environment initialized.");
    RCLCPP_INFO(this->get_logger(), "Publishing state to: %s", state_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing setpoint to: %s", setpoint_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribed to control from: %s", control_effort_topic.c_str());
  }

private:
  void timer_callback()
  {
    time_ += 0.05;  // Time increment

    // Get the simulation parameters
    double noise_level = this->get_parameter("noise_level").as_double();
    double simulation_rate = this->get_parameter("simulation_rate").as_double();

    // Simulate a simple system where state moves toward setpoint based on control_effort
    // This creates a first-order system with delay and noise
    state_ += simulation_rate * (control_effort_ + 0.1 * (setpoint_ - state_));

    // Add some noise
    double noise = noise_level * (static_cast<double>(rand()) / RAND_MAX - 0.5);

    // Publish state
    auto state_msg = std::make_unique<std_msgs::msg::Float64>();
    state_msg->data = state_ + noise;
    state_publisher_->publish(std::move(state_msg));

    // Publish setpoint
    auto setpoint_msg = std::make_unique<std_msgs::msg::Float64>();
    setpoint_msg->data = setpoint_;
    setpoint_publisher_->publish(std::move(setpoint_msg));

    RCLCPP_DEBUG(this->get_logger(), "Time: %.2f, State: %.2f, Setpoint: %.2f, Control: %.2f", time_, state_, setpoint_,
                 control_effort_);
  }

  void setpoint_change_callback()
  {
    // Alternate between two setpoints to test controller tracking
    if (setpoint_ < 3.0)
    {
      setpoint_ = 8.0;
      RCLCPP_INFO(this->get_logger(), "Changed setpoint to %.1f", setpoint_);
    }
    else
    {
      setpoint_ = 2.0;
      RCLCPP_INFO(this->get_logger(), "Changed setpoint to %.1f", setpoint_);
    }
  }

  void control_effort_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    control_effort_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received control_effort: %.4f", control_effort_);
  }

  // Publishers and subscribers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr setpoint_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr control_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;

  // State variables
  double time_;
  double setpoint_;
  double state_;
  double control_effort_{ 0.0 };
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDTestEnvironment>());
  rclcpp::shutdown();
  return 0;
}