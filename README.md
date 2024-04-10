# xbox_remote_control
This is a ros package focusing on the remote control aspect of the autonomous snowplow vehicle project. The launch file for this package runs two nodes:

1. **xbox_i2c_joy_publisher**: Custom node for reading xbox data through i2c, converting it into joy messages and publishing it to the joy topic, and
2. **teleop_twist_joy**: Node for converting the messages from the joy topic into command velocity messages published to the cmd_vel topic to be used for ros control).

## Usage
For running the launch file, use the following command:
```
ros2 launch xbox_remote_control joystick.launch.py
```
