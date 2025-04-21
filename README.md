# PID Controller Package for ROS2

This is a generic, configurable PID controller implemented as a ROS2 node in C++. It can be used to control any system that requires a PID control loop, including:

- Line following
- Altitude control
- Position control
- Velocity control
- Heading/yaw control

## Features

- Fully configurable PID parameters (P, I, D gains)
- Configurable topic names for input state, setpoint, and output control
- Adjustable output and integral anti-windup limits
- Dynamic parameter reconfiguration using the ROS2 parameter system
- Parameter sliders in rqt with appropriate ranges and precision
- Auto/manual mode switching
- Option for reverse action
- Configurable control loop rate
- Component-based architecture allowing composition

## Installation

```bash
# Clone into your ROS2 workspace src directory
cd ~/ros2_ws/src
# If you haven't already created the package
# git clone https://your-repository/pid_controller.git

# Build
cd ~/ros2_ws
colcon build --packages-select pid_controller

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

## Usage

### Basic Usage

To run a single PID controller:

```bash
ros2 run pid_controller pid_controller_standalone --ros-args \
  -p p_gain:=0.1 \
  -p i_gain:=0.01 \
  -p d_gain:=0.001 \
  -p output_min:=-1.0 \
  -p output_max:=1.0 \
  -p state_topic:=current_state \
  -p setpoint_topic:=desired_setpoint \
  -p control_effort_topic:=control_output
```

### Using Launch Files

For easier configuration, you can use the provided launch file:

```bash
ros2 launch pid_controller pid_controller.launch.py \
  p_gain:=0.1 \
  i_gain:=0.01 \
  d_gain:=0.001 \
  state_topic:=current_state
```

### Available Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `p_gain` | double | 0.0 | Proportional gain |
| `i_gain` | double | 0.0 | Integral gain |
| `d_gain` | double | 0.0 | Derivative gain |
| `output_min` | double | -1.0 | Minimum control output |
| `output_max` | double | 1.0 | Maximum control output |
| `integral_min` | double | -1.0 | Minimum integral contribution |
| `integral_max` | double | 1.0 | Maximum integral contribution (anti-windup) |
| `state_topic` | string | "state" | Topic name for incoming state measurements |
| `setpoint_topic` | string | "setpoint" | Topic name for desired setpoint |
| `control_effort_topic` | string | "control_effort" | Topic name for control output |
| `pid_enable_topic` | string | "pid_enable" | Topic name for enabling/disabling PID |
| `auto_start` | bool | true | Start in automatic mode if true |
| `reverse_action` | bool | false | Reverse control output if true |
| `publish_rate` | double | 20.0 | Control loop rate in Hz |

### Dynamic Reconfiguration

#### Using Command Line

You can adjust any parameter in real-time using the ROS2 parameter command-line interface:

```bash
# Change P gain during operation:
ros2 param set /center_pid p_gain 0.002

# Disable the controller temporarily:
ros2 param set /center_pid auto_start false

# Re-enable controller:
ros2 param set /center_pid auto_start true
```

#### Using rqt Dynamic Reconfigure

For a more user-friendly interface with sliders that allow precise floating-point values, you can use rqt:

1. Install rqt if you haven't already:
   ```bash
   sudo apt install ros-$ROS_DISTRO-rqt ros-$ROS_DISTRO-rqt-reconfigure
   ```

2. Start rqt:
   ```bash
   rqt
   ```

3. From the rqt menu, select:
   ```
   Plugins → Configuration → Dynamic Reconfigure
   ```

4. You should see your PID controller nodes in the left panel. Click on a node to expand its parameters.

5. You'll now see sliders for all float parameters, allowing you to:
   - Adjust P, I, and D gains with 0.001 precision
   - Set output and integral limits
   - Toggle boolean parameters
   - Change the control rate

6. As you adjust the sliders, the parameters will update in real-time, allowing for smooth tuning of your controller.

### Testing Dynamic Reconfiguration

To verify that the parameter sliders are working correctly with floating-point values, you can use the included test launch files that simulate a complete control environment:

1. First, make sure your workspace is built and sourced:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select pid_controller
   source install/setup.bash
   ```

2. Install the required packages if you haven't already:
   ```bash
   sudo apt update
   sudo apt install ros-$ROS_DISTRO-rqt ros-$ROS_DISTRO-rqt-reconfigure ros-$ROS_DISTRO-rqt-plot
   ```

3. Launch the basic test configuration:
   ```bash
   ros2 launch pid_controller test_dynamic_reconfigure.launch.py
   ```

   Or launch the test with visualization:
   ```bash
   ros2 launch pid_controller test_with_plot.launch.py
   ```

This will start:
- A PID controller node with default parameters
- A simulated control system that responds to the PID output
- The rqt GUI with the dynamic reconfigure plugin
- (If using test_with_plot) An rqt_plot window showing setpoint, state, and control signals

#### What to Expect

The test simulates a simple first-order control system:
- The setpoint will automatically change every 10 seconds between two values
- The simulated system will respond to the PID controller's efforts
- The PID controller will attempt to make the state track the setpoint

4. In the rqt window that opens, you should see:
   - In the left panel, expand "test_pid"
   - You'll see sliders for p_gain, i_gain, d_gain and other parameters
   - The floating-point sliders should allow precise values (e.g., 0.123)
   - Changes to parameters should be reflected in the node's output

5. Try adjusting the parameters to see their effect on control performance:
   - Increase p_gain to make the system respond faster (but may cause overshooting)
   - Add i_gain to eliminate steady-state error
   - Adjust d_gain to reduce overshooting
   - Watch the control_effort output and system response change

6. With the plot window (if using test_with_plot), you can:
   - Observe the system's response to setpoint changes
   - See how different PID parameters affect response time and stability
   - Monitor control effort to ensure it stays within reasonable bounds
   - Visualize the effects of integral wind-up and derivative kick

7. To verify changes are effective, you can check the updated parameter values:
   ```bash
   ros2 param get /test_pid p_gain
   ```

## Tips for Tuning PID Controllers

1. **Start with P**: Begin with only P gain, set I and D to zero.
2. **Tune P**: Increase P until you get reasonable response but possibly with oscillations.
3. **Add D**: Add derivative gain to reduce oscillations.
4. **Add I if needed**: Add integral gain only if there's a persistent error.
5. **Adjust limits**: Set appropriate output limits based on your system's constraints.

Remember the standard PID tuning guidelines:
- Too much P: oscillations, overshoot
- Too much I: slow response, overshoot, windup
- Too much D: noise sensitivity, jerky motion

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## References

- [ROS2 Parameter Documentation](https://docs.ros.org/en/humble/Tutorials/Parameters/Understanding-ROS2-Parameters.html)
- [PID Controller Explanation](http://wiki.ros.org/pid)
- [ROS2 rqt_reconfigure Wiki](https://wiki.ros.org/rqt_reconfigure) 