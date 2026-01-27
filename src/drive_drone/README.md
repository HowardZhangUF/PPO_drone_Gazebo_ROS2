# Drive Drone - UAV Following TurtleBot3 Project

This project demonstrates a UAV (Unmanned Aerial Vehicle) following a TurtleBot3 robot in a Gazebo simulation environment using ROS 2 Humble.

## 📋 **Overview**

The system consists of:
- **SJTU Drone**: A quadcopter UAV with position control capabilities
- **TurtleBot3 Burger**: A differential drive ground robot
- **Follower Node**: Controls the drone to follow the TurtleBot from above
- **Circle Mover Node**: Makes the TurtleBot move in a circular path

## 🎯 **Namespaces**

To avoid topic conflicts, we use separate namespaces:

| Robot | Namespace | Topics |
|-------|-----------|--------|
| **UAV** | `/simple_drone` | `/simple_drone/cmd_vel`<br>`/simple_drone/takeoff`<br>`/simple_drone/land`<br>`/simple_drone/gt_pose`<br>`/simple_drone/odom` |
| **TurtleBot3** | `/simple_robot` | `/simple_robot/cmd_vel`<br>`/simple_robot/odom`<br>`/simple_robot/scan` |

## 📁 **Project Structure**

```
drive_drone/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata
├── README.md               # This file
├── launch/
│   ├── p1_a_drone_fly_gazebo_rviz.launch.py  # Drone-only launch
│   └── p1_b_tb3_drone_gazebo.launch.py       # Main launch file (Drone + TB3)
└── src/
    ├── p1_a_drive_node.cpp        # Basic drone controller
    ├── p1_b_tb3_follower.cpp      # Vision-based follower (camera)
    ├── tb3_follower_node.cpp      # Position-based follower (odometry)
    └── tb3_circle_mover.cpp       # TurtleBot circular motion

sjtu_drone/
├── sjtu_drone_bringup/
│   ├── launch/
│   │   ├── sjtu_drone_bringup.launch.py   # Complete drone setup
│   │   └── sjtu_drone_gazebo.launch.py    # Drone + Gazebo only
│   └── config/
│       └── drone.yaml                      # Drone parameters
├── sjtu_drone_control/
│   └── sjtu_drone_control/
│       ├── simple_test.py                  # Simple takeoff/land test
│       ├── teleop.py                       # Keyboard control
│       └── drone_utils/
│           └── drone_object.py             # Drone Python API
└── sjtu_drone_description/
    ├── urdf/
    │   └── sjtu_drone.urdf.xacro          # Drone model
    ├── worlds/
    │   └── playground.world                # Gazebo world
    └── src/
        └── plugin_drone.cpp                # Drone physics plugin
```

## 🚀 **Quick Start**

### **Prerequisites**

```bash
# Ensure ROS 2 Humble is installed
source /opt/ros/humble/setup.bash

# Install TurtleBot3 packages
sudo apt update
sudo apt install -y ros-humble-turtlebot3* ros-humble-turtlebot3-gazebo

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```

### **Build the Workspace**

```bash
cd ~/ros2_ws
colcon build --packages-select drive_drone
source install/setup.bash
```

## 🎮 **Usage Scenarios**

### **Scenario 1: Autonomous Drone Following TurtleBot**

This is the main demo where the drone automatically follows a TurtleBot moving in circles.

#### Terminal 1: Launch Environment
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch drive_drone p1_b_tb3_drone_gazebo.launch.py
```

**What happens:**
- ✅ Gazebo launches with the playground world
- ✅ UAV spawns at position (0, 0, 0) with namespace `/simple_drone`
- ✅ TurtleBot3 spawns at position (3, 0, 0) with namespace `/simple_robot`
- ✅ Drone automatically takes off after 5 seconds
- 🕐 Wait ~7-10 seconds for the drone to reach altitude

#### Terminal 2: Start Drone Follower
```bash
source ~/ros2_ws/install/setup.bash
ros2 run drive_drone tb3_follower_node
```

**What it does:**
- Waits for drone to reach 2.5m altitude
- Enables position control mode
- Tracks TurtleBot3 position at 3m altitude
- Updates at 10Hz

**Expected output:**
```
[INFO] Waiting for drone to be ready...
[INFO] Sending takeoff command...
[INFO] Waiting for drone to reach altitude...
[INFO] Takeoff complete! Drone altitude: 2.85 m
[INFO] Enabling position control...
[INFO] TB3 Follower started! Drone is now tracking TurtleBot3.
[INFO] Following TB3 - Distance: 3.00 m, Drone: (3.05, 0.12, 3.00), Target: (3.00, 0.00, 3.00)
```

#### Terminal 3: Start TurtleBot Circle Mover
```bash
source ~/ros2_ws/install/setup.bash
ros2 run drive_drone tb3_circle_mover
```

**What it does:**
- Moves TurtleBot3 in a circle with 0.67m radius
- Linear velocity: 0.2 m/s
- Angular velocity: 0.3 rad/s
- Updates at 10Hz

**Expected behavior:**
- 🤖 TurtleBot moves in a circular path
- 🚁 Drone follows from 3 meters above
- 📍 Drone stays centered over the TurtleBot

---

### **Scenario 2: Manual Drone Control**

Control the drone with keyboard after manual takeoff.

#### Terminal 1: Launch Environment
```bash
ros2 launch drive_drone p1_b_tb3_drone_gazebo.launch.py
```

#### Terminal 2: Manual Takeoff
```bash
# Wait 5 seconds, then send takeoff command
ros2 topic pub -r 10 /simple_drone/takeoff std_msgs/msg/Empty "{}" -t 2
```

#### Terminal 3: Keyboard Control (Generic)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/simple_drone/cmd_vel
```

**Controls:**
- `i/j/k/l` - Forward/Left/Back/Right
- `u/o` - Rotate left/right
- `t/b` - Up/Down
- `Space` - Stop

#### OR Terminal 3: Keyboard Control (Drone-specific)
```bash
ros2 run sjtu_drone_control teleop --ros-args -r __ns:=/simple_drone
```

**Controls:**
- `w/x` - Forward/Backward
- `a/d` - Left/Right  
- `t/l` - Takeoff/Land
- `WASD` - Rotation

---

### **Scenario 3: Control TurtleBot Manually**

#### Terminal 1: Launch
```bash
ros2 launch drive_drone p1_b_tb3_drone_gazebo.launch.py
```

#### Terminal 2: TurtleBot Teleop
```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard \
  --ros-args -r cmd_vel:=/simple_robot/cmd_vel
```

**Controls:**
- `w/x` - Increase/decrease linear speed
- `a/d` - Increase/decrease angular speed
- `s` - Stop

---

### **Scenario 4: Simple Drone Test (Python)**

Test the drone with a simple takeoff/hover/land sequence.

#### Terminal 1: Launch
```bash
ros2 launch sjtu_drone_bringup sjtu_drone_gazebo.launch.py
```

#### Terminal 2: Run Test Script
```bash
cd ~/ros2_ws/src/sjtu_drone/sjtu_drone_control/sjtu_drone_control
python3 simple_test.py
```

**What it does:**
- Takes off
- Hovers for 5 seconds
- Lands

---

## 🛠️ **Troubleshooting**

### **Drone Won't Take Off**

**Symptom:** Publishing to `/simple_drone/takeoff` but nothing happens.

**Solutions:**
1. **Check if simulation is running:**
   ```bash
   ros2 topic list | grep simple_drone
   ```
   Should show `/simple_drone/takeoff`, `/simple_drone/cmd_vel`, etc.

2. **Publish repeatedly (not --once):**
   ```bash
   ros2 topic pub -r 10 /simple_drone/takeoff std_msgs/msg/Empty "{}" -t 2
   ```

3. **Check drone altitude:**
   ```bash
   ros2 topic echo /simple_drone/gt_pose
   ```

4. **Restart clean:**
   ```bash
   killall -9 gzserver gzclient
   ros2 launch drive_drone p1_b_tb3_drone_gazebo.launch.py
   ```

### **Commands Go to Wrong Robot**

**Symptom:** Publishing to `/simple_drone/cmd_vel` but TurtleBot moves instead.

**Cause:** Namespace mismatch or missing namespace remapping.

**Solution:**
- Always use **full topic names** with namespaces:
  - ✅ `/simple_drone/cmd_vel` (Drone)
  - ✅ `/simple_robot/cmd_vel` (TurtleBot)
  - ❌ `/cmd_vel` (Ambiguous - goes to first listener)

### **Build Errors**

```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select drive_drone
source install/setup.bash
```

### **Gazebo Crashes**

```bash
killall -9 gzserver gzclient
# Wait 5 seconds
ros2 launch drive_drone p1_b_tb3_drone_gazebo.launch.py
```

---

## 📚 **API Reference**

### **Drone Topics (Namespace: `/simple_drone`)**

| Topic | Type | Description |
|-------|------|-------------|
| `/simple_drone/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (must be flying) |
| `/simple_drone/takeoff` | `std_msgs/Empty` | Takeoff trigger |
| `/simple_drone/land` | `std_msgs/Empty` | Land trigger |
| `/simple_drone/reset` | `std_msgs/Empty` | Reset simulation |
| `/simple_drone/posctrl` | `std_msgs/Bool` | Enable/disable position control |
| `/simple_drone/gt_pose` | `geometry_msgs/Pose` | Ground truth position |
| `/simple_drone/odom` | `nav_msgs/Odometry` | Odometry data |
| `/simple_drone/imu` | `sensor_msgs/Imu` | IMU data |
| `/simple_drone/bottom/image_raw` | `sensor_msgs/Image` | Bottom camera |
| `/simple_drone/front/image_raw` | `sensor_msgs/Image` | Front camera |

### **TurtleBot3 Topics (Namespace: `/simple_robot`)**

| Topic | Type | Description |
|-------|------|-------------|
| `/simple_robot/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/simple_robot/odom` | `nav_msgs/Odometry` | Odometry data |
| `/simple_robot/scan` | `sensor_msgs/LaserScan` | LiDAR data |

### **Drone Control Modes**

The drone has two control modes:

1. **Velocity Mode (Default)**
   - Directly controls linear/angular velocities
   - `cmd_vel.linear.x/y/z` - Forward/Left/Up (m/s)
   - `cmd_vel.angular.z` - Yaw rate (rad/s)

2. **Position Mode**
   - Enables high-level position control
   - Must be enabled with `/simple_drone/posctrl` = True
   - `cmd_vel.linear.x/y/z` - Target X/Y/Z position (m)
   - PID controllers handle the motion

---

## 🎓 **Learning Resources**

- **ROS 2 Humble Documentation:** https://docs.ros.org/en/humble/
- **Gazebo Tutorial:** http://gazebosim.org/tutorials
- **TurtleBot3 Manual:** https://emanual.robotis.com/docs/en/platform/turtlebot3/
- **Original SJTU Drone:** https://github.com/NovoG93/sjtu_drone

---

## 📄 **License**

This project uses the GNU General Public License v3.0. See LICENSE files in individual packages for details.

---

## 🤝 **Contributing**

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

---

## 📧 **Contact**

For issues or questions, please open an issue on the GitHub repository.
