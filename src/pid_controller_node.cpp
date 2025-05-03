#include "pid_controller/pid_controller_node.hpp"

namespace pid_controller
{

PIDControllerNode::PIDControllerNode(const rclcpp::NodeOptions& node_options) : Node("pid_controller", node_options)
{
  // Define parameter descriptors with proper types and ranges for floating point controls
  rcl_interfaces::msg::ParameterDescriptor p_gain_desc;
  p_gain_desc.name = "p_gain";
  p_gain_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  p_gain_desc.description = "Proportional gain (Kp)";
  p_gain_desc.floating_point_range.resize(1);
  p_gain_desc.floating_point_range[0].from_value = 0.0;
  p_gain_desc.floating_point_range[0].to_value = 10.0;
  p_gain_desc.floating_point_range[0].step = 0.0001;

  rcl_interfaces::msg::ParameterDescriptor i_gain_desc;
  i_gain_desc.name = "i_gain";
  i_gain_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  i_gain_desc.description = "Integral gain (Ki)";
  i_gain_desc.floating_point_range.resize(1);
  i_gain_desc.floating_point_range[0].from_value = 0.0;
  i_gain_desc.floating_point_range[0].to_value = 10.0;
  i_gain_desc.floating_point_range[0].step = 0.0001;

  rcl_interfaces::msg::ParameterDescriptor d_gain_desc;
  d_gain_desc.name = "d_gain";
  d_gain_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  d_gain_desc.description = "Derivative gain (Kd)";
  d_gain_desc.floating_point_range.resize(1);
  d_gain_desc.floating_point_range[0].from_value = 0.0;
  d_gain_desc.floating_point_range[0].to_value = 10.0;
  d_gain_desc.floating_point_range[0].step = 0.0001;

  rcl_interfaces::msg::ParameterDescriptor output_min_desc;
  output_min_desc.name = "output_min";
  output_min_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  output_min_desc.description = "Minimum output value";
  output_min_desc.floating_point_range.resize(1);
  output_min_desc.floating_point_range[0].from_value = -100.0;
  output_min_desc.floating_point_range[0].to_value = 0.0;
  output_min_desc.floating_point_range[0].step = 0.001;

  rcl_interfaces::msg::ParameterDescriptor output_max_desc;
  output_max_desc.name = "output_max";
  output_max_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  output_max_desc.description = "Maximum output value";
  output_max_desc.floating_point_range.resize(1);
  output_max_desc.floating_point_range[0].from_value = 0.0;
  output_max_desc.floating_point_range[0].to_value = 100.0;
  output_max_desc.floating_point_range[0].step = 0.001;

  rcl_interfaces::msg::ParameterDescriptor integral_min_desc;
  integral_min_desc.name = "integral_min";
  integral_min_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  integral_min_desc.description = "Minimum integral term value";
  integral_min_desc.floating_point_range.resize(1);
  integral_min_desc.floating_point_range[0].from_value = -100.0;
  integral_min_desc.floating_point_range[0].to_value = 0.0;
  integral_min_desc.floating_point_range[0].step = 0.01;

  rcl_interfaces::msg::ParameterDescriptor integral_max_desc;
  integral_max_desc.name = "integral_max";
  integral_max_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  integral_max_desc.description = "Maximum integral term value";
  integral_max_desc.floating_point_range.resize(1);
  integral_max_desc.floating_point_range[0].from_value = 0.0;
  integral_max_desc.floating_point_range[0].to_value = 100.0;
  integral_max_desc.floating_point_range[0].step = 0.01;

  rcl_interfaces::msg::ParameterDescriptor publish_rate_desc;
  publish_rate_desc.name = "publish_rate";
  publish_rate_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  publish_rate_desc.description = "Control loop rate in Hz";
  publish_rate_desc.floating_point_range.resize(1);
  publish_rate_desc.floating_point_range[0].from_value = 1.0;
  publish_rate_desc.floating_point_range[0].to_value = 100.0;
  publish_rate_desc.floating_point_range[0].step = 1.0;

  // Basic descriptors for other parameters
  rcl_interfaces::msg::ParameterDescriptor auto_start_desc;
  auto_start_desc.name = "auto_start";
  auto_start_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  auto_start_desc.description = "Start in automatic mode if true";

  rcl_interfaces::msg::ParameterDescriptor reverse_action_desc;
  reverse_action_desc.name = "reverse_action";
  reverse_action_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  reverse_action_desc.description = "Reverse control output if true";

  // Declare all parameters with descriptors
  this->declare_parameter("p_gain", 0.0, p_gain_desc);
  this->declare_parameter("i_gain", 0.0, i_gain_desc);
  this->declare_parameter("d_gain", 0.0, d_gain_desc);
  this->declare_parameter("output_min", -1.0, output_min_desc);
  this->declare_parameter("output_max", 1.0, output_max_desc);
  this->declare_parameter("integral_min", -1.0, integral_min_desc);
  this->declare_parameter("integral_max", 1.0, integral_max_desc);
  this->declare_parameter("state_topic", "state");
  this->declare_parameter("setpoint_topic", "setpoint");
  this->declare_parameter("control_effort_topic", "control_effort");
  this->declare_parameter("pid_enable_topic", "pid_enable");
  this->declare_parameter("auto_start", true, auto_start_desc);
  this->declare_parameter("reverse_action", false, reverse_action_desc);
  this->declare_parameter("publish_rate", 20.0, publish_rate_desc);

  // Initialize PID controller with parameters
  init_controller();

  // Get topic names from parameters
  std::string state_topic = this->get_parameter("state_topic").as_string();
  std::string setpoint_topic = this->get_parameter("setpoint_topic").as_string();
  std::string control_effort_topic = this->get_parameter("control_effort_topic").as_string();
  std::string pid_enable_topic = this->get_parameter("pid_enable_topic").as_string();

  // Create publishers and subscribers
  control_pub_ = this->create_publisher<std_msgs::msg::Float64>(control_effort_topic, 10);
  state_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      state_topic, 10, std::bind(&PIDControllerNode::state_callback, this, std::placeholders::_1));
  setpoint_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      setpoint_topic, 10, std::bind(&PIDControllerNode::setpoint_callback, this, std::placeholders::_1));
  pid_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      pid_enable_topic, 10, std::bind(&PIDControllerNode::pid_enable_callback, this, std::placeholders::_1));

  // Set up auto mode from parameter
  auto_mode_ = this->get_parameter("auto_start").as_bool();
  reverse_action_ = this->get_parameter("reverse_action").as_bool();

  // Create timer for controller updates
  double publish_rate = this->get_parameter("publish_rate").as_double();
  timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
                                   std::bind(&PIDControllerNode::control_loop, this));

  // Set up parameter change callback
  params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&PIDControllerNode::parameters_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "PID controller node initialized with: Kp=%.4f, Ki=%.4f, Kd=%.4f",
              pid_controller_->kp, pid_controller_->ki, pid_controller_->kd);
  RCLCPP_INFO(this->get_logger(), "Listening on topics: %s (state), %s (setpoint), %s (enable)", state_topic.c_str(),
              setpoint_topic.c_str(), pid_enable_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing to: %s (control_effort)", control_effort_topic.c_str());
}

void PIDControllerNode::init_controller()
{
  double p_gain = this->get_parameter("p_gain").as_double();
  double i_gain = this->get_parameter("i_gain").as_double();
  double d_gain = this->get_parameter("d_gain").as_double();
  double output_min = this->get_parameter("output_min").as_double();
  double output_max = this->get_parameter("output_max").as_double();
  double integral_min = this->get_parameter("integral_min").as_double();
  double integral_max = this->get_parameter("integral_max").as_double();

  // Create PID controller with parameters
  pid_controller_ =
      std::make_unique<PID>(p_gain, i_gain, d_gain, 0.0,  // setpoint starts at 0
                            std::make_pair(output_min, output_max), std::make_pair(integral_min, integral_max));
}

void PIDControllerNode::state_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  current_state_ = msg->data;
  has_received_state_ = true;
}

void PIDControllerNode::setpoint_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  if (current_setpoint_ != msg->data)
  {
    current_setpoint_ = msg->data;
    pid_controller_->set_setpoint(current_setpoint_);
    has_received_setpoint_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Setpoint updated to %f", current_setpoint_);
  }
}

void PIDControllerNode::pid_enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  auto_mode_ = msg->data;

  if (!auto_mode_)
  {
    // Reset controller when disabling
    pid_controller_->reset();
  }

  RCLCPP_INFO(this->get_logger(), "PID controller %s", auto_mode_ ? "enabled" : "disabled");
}

void PIDControllerNode::control_loop()
{
  if (!auto_mode_)
  {
    return;  // Don't update if in manual mode
  }

  if (!has_received_state_)
  {
    RCLCPP_DEBUG(this->get_logger(), "Waiting for state input...");
    return;
  }

  if (!has_received_setpoint_)
  {
    RCLCPP_DEBUG(this->get_logger(), "Waiting for setpoint...");
    return;
  }

  // Calculate control output
  double control_effort = pid_controller_->update(current_state_);

  // Apply reverse action if configured
  if (reverse_action_)
  {
    control_effort = -control_effort;
  }

  // Publish control effort
  auto msg = std::make_unique<std_msgs::msg::Float64>();
  msg->data = control_effort;
  control_pub_->publish(std::move(msg));
}

rcl_interfaces::msg::SetParametersResult
PIDControllerNode::parameters_callback(const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try
  {
    for (const auto& param : parameters)
    {
      if (param.get_name() == "p_gain")
      {
        pid_controller_->kp = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated p_gain to %.4f", param.as_double());
      }
      else if (param.get_name() == "i_gain")
      {
        pid_controller_->ki = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated i_gain to %.4f", param.as_double());
      }
      else if (param.get_name() == "d_gain")
      {
        pid_controller_->kd = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated d_gain to %.4f", param.as_double());
      }
      else if (param.get_name() == "output_min")
      {
        pid_controller_->output_limits.first = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated output_min to %.4f", param.as_double());
      }
      else if (param.get_name() == "output_max")
      {
        pid_controller_->output_limits.second = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated output_max to %.4f", param.as_double());
      }
      else if (param.get_name() == "integral_min")
      {
        pid_controller_->integral_limits.first = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated integral_min to %.4f", param.as_double());
      }
      else if (param.get_name() == "integral_max")
      {
        pid_controller_->integral_limits.second = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated integral_max to %.4f", param.as_double());
      }
      else if (param.get_name() == "auto_start")
      {
        auto_mode_ = param.as_bool();
        if (!auto_mode_)
        {
          pid_controller_->reset();  // Reset when going to manual mode
        }
        RCLCPP_INFO(this->get_logger(), "PID controller %s", auto_mode_ ? "enabled" : "disabled");
      }
      else if (param.get_name() == "reverse_action")
      {
        reverse_action_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Reverse action %s", reverse_action_ ? "enabled" : "disabled");
      }
      else if (param.get_name() == "publish_rate")
      {
        double publish_rate = param.as_double();
        // Update the timer period
        timer_->cancel();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
                                         std::bind(&PIDControllerNode::control_loop, this));
        RCLCPP_INFO(this->get_logger(), "Updated publish rate to %.2f Hz", publish_rate);
      }
    }
  }
  catch (const std::exception& e)
  {
    result.successful = false;
    result.reason = std::string("Error updating parameters: ") + e.what();
    RCLCPP_ERROR(this->get_logger(), "Error in parameter callback: %s", e.what());
  }

  return result;
}

}  // namespace pid_controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pid_controller::PIDControllerNode)
