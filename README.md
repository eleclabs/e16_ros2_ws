# Omni Robot ROS 2 Jazzy Workspace

Workspace สำหรับพัฒนา **Omni 3-Wheel / Kiwi Drive Robot** บน ROS 2 Jazzy / Ubuntu 24.04 โดยรัน ROS 2 ภายใน Docker และ bind-mount workspace จาก WSL:

```text
Windows 11
  │
  └── WSL2 Ubuntu 24.04
        │
        └── ~/ros2_ws
              │ bind mount
              ▼
        Docker: ros2_jazzy
              │
              └── /ros2_ws
```

## Environment

- ROS 2: Jazzy
- OS: Ubuntu 24.04
- Container: `ros2_jazzy`
- Workspace in container: `/ros2_ws`
- WSL workspace: `~/ros2_ws`
- Middleware: `rmw_cyclonedds_cpp`
- DDS configuration: `/ros2_ws/config/cyclonedds.xml`

ตรวจสอบ mount:

```bash
docker inspect ros2_jazzy --format '{{range .Mounts}}{{println .Source "->" .Destination}}{{end}}'
```

ควรมี:

```text
/home/eleclabs/ros2_ws -> /ros2_ws
```

---

# 1. Workspace Architecture

```text
ros2_ws
├── src
│   ├── omni_robot_description
│   ├── omni_robot_bringup
│   ├── omni_robot_controller
│   ├── omni_robot_navigation
│   ├── omni_robot_localization
│   └── robot_sim_bridge
├── config
│   └── cyclonedds.xml
├── build
├── install
└── log
```

## Package Responsibilities

### `omni_robot_description`
จัดการ URDF/Xacro, links/joints, wheels, LiDAR, IMU, RViz และ robot description TF

### `omni_robot_controller`
จัดการ `/cmd_vel`, 3-wheel omni kinematics, motor/encoder hardware abstraction และ wheel odometry

### `omni_robot_localization`
รวม wheel odometry + IMU ผ่าน EKF และจัดการ localization TF

### `omni_robot_navigation`
SLAM Toolbox, map, Nav2 planner/controller และ costmaps

### `omni_robot_bringup`
รวม launch สำหรับระบบจริง

### `robot_sim_bridge`
เชื่อม ROS 2 กับ simulator

---

# 2. Omni 3-Wheel Geometry

กำหนดทิศทางของหุ่นยนต์ดังนี้:

```text
                    FRONT (+X)
                         ↑
                         │
             W1          │          W2
        Front Left       │      Front Right
             ●           │           ●
              \          │          /
               \      base_link   /
                \        ●        /
                 \               /
                  └──────┬──────┘
                         │
                         ●
                         W3
                     Rear Center

+X = ด้านหน้า
-X = ด้านหลัง
+Y = ด้านซ้าย
-Y = ด้านขวา
```

Wheel order **ต้องเหมือนกันทุก package**:

```text
[0] Wheel 1 = Front Left
[1] Wheel 2 = Front Right
[2] Wheel 3 = Rear Center
```

ค่าเริ่มต้น:

```text
wheel_radius      = 0.032 m
wheel_base_radius = 0.115 m
wheel angles      = +60°, -60°, 180°
```

---

# 3. Controller Architecture

```text
                         /cmd_vel
                            │
                            ▼
                 ┌────────────────────┐
                 │ omni_kinematics    │
                 │      node          │
                 └─────────┬──────────┘
                           │
                           ▼
                /wheel_velocity_cmd
                   [W1, W2, W3]
                           │
                           ▼
                ┌────────────────────┐
                │ hardware interface │
                └─────────┬──────────┘
                          │
                ┌─────────┴─────────┐
                ▼                   ▼
             Motors             Encoders
                │                   │
                │                   ▼
                │       /wheel_velocity_feedback
                │             [W1, W2, W3]
                │                   │
                └──────────┬────────┘
                           ▼
                 ┌──────────────────┐
                 │ omni_odometry    │
                 │      node        │
                 └────────┬─────────┘
                          │
                    ┌─────┴─────┐
                    ▼           ▼
                  /odom     odom→base_link
```

---

# 4. Controller Files

```text
src/omni_robot_controller
├── omni_robot_controller
│   ├── __init__.py
│   ├── omni_kinematics_node.py
│   ├── omni_odometry_node.py
│   └── omni_hardware_interface.py
├── config
│   └── controller.yaml
├── launch
│   └── controller.launch.py
├── resource
│   └── omni_robot_controller
├── setup.py
├── setup.cfg
└── package.xml
```

## Kinematics

Input:

```text
/cmd_vel
```

Output:

```text
/wheel_velocity_cmd
```

ชนิดข้อความ:

```text
std_msgs/msg/Float64MultiArray
```

ลำดับ:

```text
data[0] = W1 Front Left
 data[1] = W2 Front Right
 data[2] = W3 Rear Center
```

สมการที่ใช้:

```text
v_wheel = -sin(theta) * Vx
         + cos(theta) * Vy
         + R * Wz

omega_wheel = v_wheel / wheel_radius
```

---

# 5. Hardware Interface — Phase 3

`omni_hardware_interface.py` ถูกเพิ่มเพื่อแยก hardware layer ออกจาก kinematics และ odometry

โหมดปัจจุบัน:

```yaml
simulation_mode: true
```

เมื่อเป็น `true` node จะใช้ `/wheel_velocity_cmd` เป็น feedback เพื่อทดสอบระบบแบบ open-loop

**นี่ไม่ใช่ encoder feedback จริง**

สำหรับ hardware จริงต้องเปลี่ยนเป็น:

```text
ROS 2
  │
  └── /wheel_velocity_cmd
          │
          ▼
    micro-ROS Agent
          │ UDP/Serial
          ▼
        ESP32
       ┌──┼──┐
       ▼  ▼  ▼
      M1  M2 M3
       │  │  │
      E1  E2 E3
       └──┼──┘
          ▼
/wheel_velocity_feedback
```

โดย ESP32 ต้องส่ง **encoder-derived angular velocity [rad/s]** กลับมาในลำดับ W1/W2/W3

---

# 6. Controller Parameters

ไฟล์:

```text
src/omni_robot_controller/config/controller.yaml
```

ค่าหลัก:

```yaml
wheel_radius: 0.032
wheel_base_radius: 0.115
command_timeout: 0.5
max_wheel_velocity: 20.0
publish_rate: 50.0
simulation_mode: true
```

`command_timeout` เป็น safety watchdog เมื่อไม่มี command ใหม่

---

# 7. Build

เข้า Docker:

```bash
docker exec -it ros2_jazzy bash
```

จากนั้น:

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select omni_robot_controller --symlink-install
source /ros2_ws/install/setup.bash
```

ถ้าต้องการ clean เฉพาะ package:

```bash
rm -rf build/omni_robot_controller
colcon build --packages-select omni_robot_controller --symlink-install
```

---

# 8. Verify Controller

ตรวจ executable:

```bash
ros2 pkg executables omni_robot_controller
```

Expected:

```text
omni_robot_controller omni_kinematics_node
omni_robot_controller omni_odometry_node
omni_robot_controller omni_hardware_interface
```

ตรวจ package:

```bash
ros2 pkg prefix omni_robot_controller
```

ตรวจ launch:

```bash
ros2 launch omni_robot_controller controller.launch.py
```

---

# 9. Controller Test

เปิด controller ทั้งชุด:

```bash
ros2 launch omni_robot_controller controller.launch.py
```

ตรวจ node:

```bash
ros2 node list
```

ควรมี:

```text
/omni_kinematics_node
/omni_hardware_interface
/omni_odometry_node
```

ตรวจ topics:

```bash
ros2 topic list | grep -E 'cmd_vel|wheel|odom'
```

ควรมี:

```text
/cmd_vel
/wheel_velocity_cmd
/wheel_velocity_feedback
/odom
```

---

# 10. Test Forward Motion

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

ตรวจ wheel command:

```bash
ros2 topic echo /wheel_velocity_cmd
```

ตรวจ feedback:

```bash
ros2 topic echo /wheel_velocity_feedback
```

ตรวจ odometry:

```bash
ros2 topic echo /odom
```

---

# 11. Test Lateral Motion

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Omni robot ต้องสามารถเคลื่อนที่ด้านข้างได้โดยไม่ต้องหมุนตัว

---

# 12. Test Rotation

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

ตรวจ:

```bash
ros2 topic echo /wheel_velocity_cmd
```

ต้องตรวจ sign ของ W1/W2/W3 ก่อนต่อมอเตอร์จริง

---

# 13. Safety Test

ส่ง command หนึ่งครั้ง:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

หลังเกิน `command_timeout` ควรเห็น wheel command เป็นศูนย์:

```text
[0.0, 0.0, 0.0]
```

---

# 14. Odometry / TF

ตรวจ:

```bash
ros2 topic echo /odom
```

และ:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

TF ที่ controller รับผิดชอบในช่วงนี้:

```text
odom
  │
  └── base_link
```

เมื่อเพิ่ม EKF ภายหลัง ต้องไม่ให้ node หลายตัว publish transform เดียวกันพร้อมกัน

---

# 15. Important Integration Rule

ต้องรักษา wheel order เดียวกันทั้งระบบ:

```text
URDF
  ↓
Controller
  ↓
Motor Driver
  ↓
Encoder
  ↓
Odometry
  ↓
Localization
  ↓
Nav2

W1 = Front Left
W2 = Front Right
W3 = Rear Center
```

ถ้า wheel sign หรือ wheel order ผิด จะเกิดอาการ เช่น:

```text
สั่งเดินหน้า → หุ่นยนต์หมุน
สั่งซ้าย     → หุ่นยนต์ไปขวา
สั่งหมุน     → หุ่นยนต์เคลื่อนที่
```

ดังนั้นต้องทดสอบล้อโดยยกล้อพ้นพื้นก่อนต่อ floor test

---

# 16. Current Development Status

| Phase | System | Status |
|---|---|---|
| 1 | `omni_robot_description` | Implemented |
| 1.1 | URDF/Xacro + 3-wheel layout | Implemented |
| 1.2 | RViz / TF description | Implemented |
| 2 | `omni_robot_controller` | Implemented |
| 2.1 | `/cmd_vel` → wheel kinematics | Implemented |
| 2.2 | Wheel feedback → `/odom` | Implemented |
| 3 | Hardware abstraction | Implemented for simulation/open-loop |
| 3.1 | Real ESP32/micro-ROS motor driver | Pending |
| 3.2 | Real encoder feedback | Pending |
| 4 | `omni_robot_localization` + EKF | Pending |
| 5 | SLAM Toolbox | Pending |
| 6 | Save/Load Map | Pending |
| 7 | Nav2 | Pending |
| 8 | `omni_robot_bringup` integration | Pending |
| 9 | `robot_sim_bridge` integration | Pending |
| 10 | Full robot test | Pending |

---

# 17. Phase 3A — Controller Validation

ก่อนต่อ hardware จริง ให้ validate kinematics และ safety ใน simulation/open-loop ก่อน

## Automated Kinematics Tests

ไฟล์ทดสอบ:

```text
src/omni_robot_controller/test/test_omni_kinematics.py
```

รันทดสอบ:

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon test --packages-select omni_robot_controller --event-handlers console_direct+
colcon test-result --verbose
```

สิ่งที่ทดสอบ:

```text
Forward       vx=0.1 m/s
Lateral       vy=0.1 m/s
Rotation      wz=0.5 rad/s
Velocity limit 20 rad/s
Zero limit    emergency stop behavior
```

## Wheel Velocity Limit

`omni_kinematics_node` มี `max_wheel_velocity` และจะ scale ล้อทั้งหมดพร้อมกันเมื่อมีล้อใดเกิน limit เพื่อรักษา direction/ratio ของ motion

```yaml
max_wheel_velocity: 20.0
```

การ scale แบบนี้ดีกว่าการ clip แต่ละล้อแยกกัน เพราะไม่ทำให้ทิศทางการเคลื่อนที่ของ Omni robot เปลี่ยน

## Manual Validation

เปิด controller:

```bash
ros2 launch omni_robot_controller controller.launch.py
```

ตรวจ:

```bash
ros2 topic echo /wheel_velocity_cmd
ros2 topic echo /wheel_velocity_feedback
ros2 topic echo /odom
ros2 run tf2_ros tf2_echo odom base_link
```

ทดสอบ Forward:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \\
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

ทดสอบ Lateral:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \\
  "{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

ทดสอบ Rotation:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \\
  "{linear: {x: 0.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

หลังหยุดส่ง command เกิน `0.5 s` wheel command ต้องกลับเป็น:

```text
[0.0, 0.0, 0.0]
```

**ห้ามต่อ motor จริงจนกว่า Phase 3A จะผ่าน**

---

# 18. Phase 3.1 — ESP32-S3 + micro-ROS Hardware Driver

Firmware ถูกจัดเตรียมไว้ที่:

```text
esp32/omni_micro_ros/
```

Hardware target:

```text
ESP32-S3
3 × TB6612
3 × DC Motor 12 V / 200 RPM
3 × Quadrature Encoder 500 P/R
```

Firmware features:

```text
micro-ROS UDP
ESP-IDF PCNT quadrature encoder
3 independent velocity PID loops
500 ms command watchdog
/wheel_velocity_cmd
/wheel_velocity_feedback
```

เริ่มต้น firmware:

```bash
cd /ros2_ws/esp32/omni_micro_ros
pio run
pio run -t upload
pio device monitor -b 115200
```

ตั้งค่า Wi-Fi และ micro-ROS Agent ใน `src/main.cpp` ก่อน upload:

```cpp
WIFI_SSID
WIFI_PASSWORD
AGENT_IP
AGENT_PORT
```

บน ROS 2:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

ตรวจ feedback:

```bash
ros2 topic echo /wheel_velocity_feedback
```

ทดสอบล้อทีละล้อโดยยกล้อพ้นพื้น:

```bash
ros2 topic pub --once /wheel_velocity_cmd std_msgs/msg/Float64MultiArray \\
  "{data: [2.0, 0.0, 0.0]}"
```

ก่อน floor test ต้องตรวจ encoder resolution, motor sign, encoder sign และ PID ให้ผ่านก่อน

รายละเอียดทั้งหมดอยู่ที่:

```text
esp32/omni_micro_ros/README.md
```

ขั้นถัดไปคือ **Phase 3.2: Real Encoder Validation + PID Tuning + `/odom` integration**

## Phase 3.4 — EKF + MPU6050

เพิ่ม package:

```text
src/omni_robot_localization/
├── config/ekf.yaml
└── launch/ekf.launch.py
```

EKF จะ fuse:

```text
/wheel_velocity_feedback
        ↓
omni_odometry_node
        ↓
/odom
        ┐
        ├── robot_localization EKF
/imu/data_raw ─┘
        ↓
/odometry/filtered
        ↓
odom → base_link
```

MPU6050 ใช้เฉพาะ `angular_velocity.z` ใน EKF เพราะ MPU6050 ไม่มี magnetometer และ firmware ยังไม่มี absolute yaw orientation.

เริ่ม EKF หลัง build:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch omni_robot_localization ekf.launch.py
```

ตรวจ:

```bash
ros2 topic echo /odometry/filtered
ros2 topic hz /odometry/filtered
ros2 run tf2_ros tf2_echo odom base_link
```

**ก่อนใช้งานจริงต้องติดตั้ง `robot_localization` ใน ROS 2 Jazzy และ build workspace ใหม่**

ขั้นถัดไปหลัง EKF คือ **Phase 3.5: IMU axis calibration + covariance tuning + real floor test**

เป้าหมาย:

```text
                ROS 2 Jazzy
                    │
              /wheel_velocity_cmd
                    │
                    ▼
             micro-ROS Agent
                    │
                   UDP
                    │
                    ▼
                  ESP32
          ┌─────────┼─────────┐
          ▼         ▼         ▼
       Motor 1   Motor 2   Motor 3
       Encoder1  Encoder2  Encoder3
          └─────────┼─────────┘
                    ▼
          /wheel_velocity_feedback
                    │
                    ▼
                /odom
                    │
                    ▼
             robot_localization
                    │
                    ▼
               SLAM Toolbox
                    │
                    ▼
                  Nav2
```

ก่อนทำ EKF/SLAM/Nav2 ต้องทำให้ motor command, encoder feedback, wheel sign และ odometry ถูกต้องบน hardware จริงก่อน

---

# 19. Clean Build Guidance

ไม่ควรลบ `build`, `install`, `log` ทุกครั้ง เพราะทำให้ build ใหม่ทั้งหมด

แนะนำ:

```bash
# เปลี่ยน package เดียว
rm -rf build/<package_name>
colcon build --packages-select <package_name> --symlink-install
```

Clean ทั้ง workspace เมื่อจำเป็นจริง ๆ:

```bash
rm -rf build install log
colcon build --symlink-install
```

---

# 20. Phase 3.5 — Hardware Calibration

ก่อนลงพื้นจริง ให้ทำตามลำดับใน `docs/PHASE_3_5_HARDWARE_CALIBRATION.md`:

```text
ESP32 connection
      ↓
MPU6050 I2C + gyro bias
      ↓
Encoder direction + counts/rev
      ↓
Single-wheel PID
      ↓
3-wheel velocity tracking
      ↓
Omni odometry
      ↓
EKF
      ↓
TF validation
```

ห้ามใช้ `use_simulation:=true` ร่วมกับ ESP32 จริง เพราะ simulation interface จะกลายเป็น publisher ของ `/wheel_velocity_feedback` อีกตัว

สำหรับ real hardware:

```bash
ros2 launch omni_robot_localization localization.launch.py
```

สำหรับ simulation เท่านั้น:

```bash
ros2 launch omni_robot_localization localization.launch.py use_simulation:=true
```

## 21. Project Principle

โปรเจกต์นี้จะแยกความรับผิดชอบเป็นชั้น ๆ เพื่อให้สามารถย้าย hardware ระหว่าง simulation, ESP32, Raspberry Pi และ ROS 2 PC ได้โดยไม่ต้องเปลี่ยน kinematics/navigation ทั้งระบบ:

```text
Description Layer
       ↓
Control / Kinematics Layer
       ↓
Hardware Interface Layer
       ↓
Localization Layer
       ↓
Mapping Layer
       ↓
Navigation Layer
       ↓
Bringup / Application Layer
```
