from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    """
    Test launch file for verifying dynamic reconfiguration with rqt and visualizing
    the PID controller's performance with rqt_plot.

    This launches:
    1. A basic PID controller node with default parameters
    2. A test environment node that simulates a control system
    3. RQT with the dynamic reconfigure plugin
    4. RQT Plot to visualize the system behavior
    """
    return LaunchDescription(
        [
            # PID controller node
            Node(
                package="pid_controller",
                executable="pid_controller_standalone",
                name="test_pid",
                output="screen",
                parameters=[
                    {
                        "p_gain": 0.5,
                        "i_gain": 0.1,
                        "d_gain": 0.05,
                        "output_min": -2.0,
                        "output_max": 2.0,
                        "state_topic": "/test/state",
                        "setpoint_topic": "/test/setpoint",
                        "control_effort_topic": "/test/control_effort",
                    }
                ],
            ),
            # Test environment - simulates a system to control
            Node(
                package="pid_controller",
                executable="pid_test_environment",
                name="test_environment",
                output="screen",
                parameters=[
                    {
                        "state_topic": "/test/state",
                        "setpoint_topic": "/test/setpoint",
                        "control_effort_topic": "/test/control_effort",
                        "publish_frequency": 20.0,
                        "noise_level": 0.1,
                        "simulation_rate": 0.2,
                    }
                ],
            ),
            # Launch rqt_reconfigure directly using ExecuteProcess
            ExecuteProcess(
                cmd=["ros2", "run", "rqt_reconfigure", "rqt_reconfigure"],
                output="screen",
            ),
            # RQT Plot for visualization
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "rqt_plot",
                    "rqt_plot",
                    "/test/state/data",
                    "/test/setpoint/data",
                    "/test/control_effort/data",
                ],
                output="screen",
            ),
        ]
    )
