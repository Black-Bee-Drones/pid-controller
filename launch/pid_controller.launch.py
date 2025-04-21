from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare all the launch arguments
    p_gain = LaunchConfiguration("p_gain")
    i_gain = LaunchConfiguration("i_gain")
    d_gain = LaunchConfiguration("d_gain")
    output_min = LaunchConfiguration("output_min")
    output_max = LaunchConfiguration("output_max")
    integral_min = LaunchConfiguration("integral_min")
    integral_max = LaunchConfiguration("integral_max")
    state_topic = LaunchConfiguration("state_topic")
    setpoint_topic = LaunchConfiguration("setpoint_topic")
    control_effort_topic = LaunchConfiguration("control_effort_topic")
    pid_enable_topic = LaunchConfiguration("pid_enable_topic")
    auto_start = LaunchConfiguration("auto_start")
    reverse_action = LaunchConfiguration("reverse_action")
    publish_rate = LaunchConfiguration("publish_rate")
    node_name = LaunchConfiguration("node_name")

    return LaunchDescription(
        [
            # Declare all launch parameters with default values
            DeclareLaunchArgument(
                "p_gain", default_value="0.0", description="Proportional gain (Kp)"
            ),
            DeclareLaunchArgument(
                "i_gain", default_value="0.0", description="Integral gain (Ki)"
            ),
            DeclareLaunchArgument(
                "d_gain", default_value="0.0", description="Derivative gain (Kd)"
            ),
            DeclareLaunchArgument(
                "output_min", default_value="-1.0", description="Minimum output value"
            ),
            DeclareLaunchArgument(
                "output_max", default_value="1.0", description="Maximum output value"
            ),
            DeclareLaunchArgument(
                "integral_min",
                default_value="-1.0",
                description="Minimum integral term value",
            ),
            DeclareLaunchArgument(
                "integral_max",
                default_value="1.0",
                description="Maximum integral term value",
            ),
            DeclareLaunchArgument(
                "state_topic",
                default_value="state",
                description="Topic name for state input",
            ),
            DeclareLaunchArgument(
                "setpoint_topic",
                default_value="setpoint",
                description="Topic name for setpoint input",
            ),
            DeclareLaunchArgument(
                "control_effort_topic",
                default_value="control_effort",
                description="Topic name for control output",
            ),
            DeclareLaunchArgument(
                "pid_enable_topic",
                default_value="pid_enable",
                description="Topic name for enabling/disabling the PID controller",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                description="Whether to start in auto mode (true) or manual mode (false)",
            ),
            DeclareLaunchArgument(
                "reverse_action",
                default_value="false",
                description="Whether to reverse the control action",
            ),
            DeclareLaunchArgument(
                "publish_rate",
                default_value="20.0",
                description="Control loop rate in Hz",
            ),
            DeclareLaunchArgument(
                "node_name",
                default_value="pid_controller",
                description="Name of the PID controller node",
            ),
            # Create the PID controller node with parameters
            Node(
                package="pid_controller",
                executable="pid_controller_standalone",
                name=node_name,
                output="screen",
                parameters=[
                    {
                        "p_gain": p_gain,
                        "i_gain": i_gain,
                        "d_gain": d_gain,
                        "output_min": output_min,
                        "output_max": output_max,
                        "integral_min": integral_min,
                        "integral_max": integral_max,
                        "state_topic": state_topic,
                        "setpoint_topic": setpoint_topic,
                        "control_effort_topic": control_effort_topic,
                        "pid_enable_topic": pid_enable_topic,
                        "auto_start": auto_start,
                        "reverse_action": reverse_action,
                        "publish_rate": publish_rate,
                    }
                ],
            ),
        ]
    )
