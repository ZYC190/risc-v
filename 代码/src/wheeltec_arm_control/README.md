# wheeltec_arm_control

ROS 2 Python package containing the WHEELTEC arm serial driver and the
vision-based grabber node.

```bash
ros2 run wheeltec_arm_control arm_control
```

The combined node handles vision grabbing, `joint_states`, and `arm_teleop`
through one serial connection. The legacy standalone commands remain available:

```bash
ros2 run wheeltec_arm_control arm_serial_driver
ros2 run wheeltec_arm_control arm_grabber
```

Both nodes open `/dev/wheeltec_arm`, so run only one of them at a time.
