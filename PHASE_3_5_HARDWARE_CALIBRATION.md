# Phase 3.5 — Hardware Calibration

## Target
ESP32-S3 + 3 × TB6612 + 3 × 12 V / 200 RPM motors + 500 P/R quadrature encoders + MPU6050 + ROS 2 Jazzy.

## Safety

- First tests: lift all wheels off the floor.
- Use a current-limited 12 V supply when possible.
- Keep an emergency power disconnect available.
- Do not change motor direction wiring while power is applied.
- Do not run `use_simulation:=true` with the real ESP32 because it creates a second `/wheel_velocity_feedback` publisher.

## 1. Build ROS 2 workspace

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select omni_robot_controller omni_robot_localization --symlink-install
source install/setup.bash
```

If `robot_localization` is missing:

```bash
sudo apt update
sudo apt install ros-jazzy-robot-localization
```

## 2. Build ESP32 firmware

```bash
cd ~/ros2_ws/esp32/omni_micro_ros
pio run
pio run -t upload
pio device monitor -b 115200
```

Keep the robot stationary during MPU6050 gyro calibration at boot.

Expected startup messages include:

```text
MPU6050 ready. gyro bias [...]
micro-ROS time synchronized
```

## 3. Encoder counts/revolution

Current firmware assumption:

```text
500 P/R/channel × quadrature x4 = 2000 counts/rev
```

If the encoder specification means 500 CPR after quadrature decoding, change:

```cpp
ENCODER_PPR = 500.0f;
QUADRATURE_COUNTS_PER_REV = 500.0f;
```

Validation method:

1. Disable motor power.
2. Rotate one wheel exactly one mechanical revolution.
3. Inspect encoder diagnostics.
4. Confirm approximately 2000 counts with the current assumption.
5. Repeat for all three wheels.

The firmware clears PCNT after every 10 ms sample, preventing the 16-bit counter from accumulating until overflow.

## 4. Encoder direction

Positive wheel command must produce positive wheel feedback.

```bash
ros2 topic pub --once /wheel_velocity_cmd std_msgs/msg/Float64MultiArray "{data: [2.0, 0.0, 0.0]}"
ros2 topic pub --once /wheel_velocity_cmd std_msgs/msg/Float64MultiArray "{data: [0.0, 2.0, 0.0]}"
ros2 topic pub --once /wheel_velocity_cmd std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 2.0]}"
```

Inspect:

```bash
ros2 topic echo /wheel_velocity_feedback
```

If a wheel reports the opposite sign, change only its `ENCODER_SIGN[i]`. If the motor itself rotates opposite to the expected positive direction, change only its `MOTOR_SIGN[i]`. Do not fix a motor polarity error by changing EKF configuration.

## 5. MPU6050 axis test

```text
VCC → 3.3 V
GND → GND
SDA → GPIO1
SCL → GPIO2
AD0 → GND (0x68)
```

Check:

```bash
ros2 topic hz /imu/data_raw
ros2 topic echo /imu/data_raw
```

Target rate: `~100 Hz`.

At rest, gyro should be close to zero after bias calibration. Acceleration magnitude should be close to 9.81 m/s².

Rotate the robot counter-clockwise around physical Z. Positive yaw rotation should produce positive `angular_velocity.z`.

If the sign is wrong, verify physical IMU orientation and `imu_link` TF first. Do not compensate by changing EKF signs until the physical frame is correct.

## 6. Single-wheel PID tuning

Start:

```text
Kp = 0.55
Ki = 1.80
Kd = 0
```

With wheels lifted, test approximately `2`, `5`, and `10 rad/s` and record command, measured velocity, steady-state error, overshoot and settling time. Tune one wheel at a time. If oscillating, reduce `Ki` first; if slow, increase `Kp` gradually.

## 7. Kinematics checks

Current geometry:

```text
wheel_radius      = 0.032 m
wheel_base_radius = 0.115 m
```

These must match the physical robot.

For `vx=0.10 m/s, vy=0, wz=0`, expected wheel commands are approximately:

```text
W1 = -2.706 rad/s
W2 = +2.706 rad/s
W3 =  0.000 rad/s
```

For `vx=0, vy=0.10 m/s, wz=0`:

```text
W1 = +1.563 rad/s
W2 = +1.563 rad/s
W3 = -3.125 rad/s
```

For `vx=0, vy=0, wz=0.5 rad/s`:

```text
W1 = W2 = W3 ≈ +1.797 rad/s
```

## 8. Start real localization

Terminal 1:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Terminal 2:

```bash
ros2 launch omni_robot_localization localization.launch.py
```

Verify publisher ownership:

```bash
ros2 topic info /wheel_velocity_feedback -v
```

Real hardware should have the ESP32 node as publisher. There must not be an `omni_hardware_interface` publisher.

## 9. EKF checks

```bash
ros2 topic hz /odom
ros2 topic hz /imu/data_raw
ros2 topic hz /odometry/filtered
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link imu_link
```

Expected approximately:

```text
/odom                ~20 Hz
/imu/data_raw        ~100 Hz
/odometry/filtered   ~50 Hz
```

Only EKF should publish `odom → base_link`. `robot_state_publisher`/URDF should provide the fixed `base_link → imu_link` transform.

## 10. Static test

Leave the robot stationary for 60 seconds. `/imu/data_raw` gyro should remain near zero and `/odometry/filtered` should not drift rapidly. If yaw drifts, check gyro bias, IMU mounting, covariance and timestamps before changing EKF settings.

## 11. Straight-line test

Command low speed:

```text
vx = 0.05 m/s
vy = 0
wz = 0
```

Check final x/y/yaw. Use measured distance to refine `wheel_radius`.

## 12. Rotation test

Command:

```text
vx = 0
vy = 0
wz = 0.2 rad/s
```

All three wheel feedback values should have the same sign and approximately the same magnitude. Use a measured 360° rotation to refine `wheel_base_radius`.

## 13. Pass criteria

- [ ] ESP32 connects to micro-ROS Agent.
- [ ] `/wheel_velocity_feedback` has exactly one publisher.
- [ ] Three encoders have correct sign.
- [ ] Counts/revolution are verified.
- [ ] PID tracks velocity without sustained oscillation.
- [ ] MPU6050 publishes at ~100 Hz.
- [ ] Gyro bias is stable.
- [ ] IMU Z sign matches CCW rotation.
- [ ] `/odom` is stable at rest.
- [ ] `/odometry/filtered` is stable at rest.
- [ ] `odom → base_link` has exactly one TF authority.
- [ ] `base_link → imu_link` is correct.
- [ ] Straight motion does not create large yaw drift.
- [ ] Rotation test gives approximately correct angular displacement.

## 14. Next phase

After this checklist passes:

```text
Phase 4  RPLIDAR /scan validation
   ↓
Phase 4.1 SLAM Toolbox
   ↓
Phase 4.2 Save map
   ↓
Phase 5 Nav2
   ↓
Phase 5.1 Goal Pose
   ↓
Phase 5.2 Path + Costmap
   ↓
Phase 5.3 Autonomous Omni navigation
```
