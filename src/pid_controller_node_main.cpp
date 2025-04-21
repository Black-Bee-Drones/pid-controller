#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "pid_controller/pid_controller_node.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Create a PID controller node with default node options
  rclcpp::spin(std::make_shared<pid_controller::PIDControllerNode>());

  rclcpp::shutdown();
  return 0;
}