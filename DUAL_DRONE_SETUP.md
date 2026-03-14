# Dual Drone Setup Guide

This document explains how to run multiple independent drones simultaneously in Gazebo with proper namespace isolation.

## Quick Start

```bash
cd ~/PPO_drone_Gazebo_ROS2
colcon build --packages-select sjtu_drone_description sjtu_drone_control drive_drone
source install/setup.bash
ros2 launch drive_drone drone_dual_tb3_gazebo.launch.py
```

## Problem Statement

The original sjtu_drone package was designed for single-drone operation. Running multiple drones caused several issues:

1. **Spawn Position**: Both drones spawned at the same location (0, 0, 0)
2. **Physics Collision**: Drones fell through the ground due to missing collision/physics data
3. **Namespace Pollution**: Hardcoded `/simple_drone` namespace in SDF files prevented proper isolation
4. **Physics Hijacking**: C++ plugin used global world lookup, causing drone2's commands to control drone1's body

## Solution Overview

The solution required changes across 5 files in the stack:

1. **spawn_drone.py** - Accept position arguments
2. **drone_dual_tb3_gazebo.launch.py** - Process xacro with physics, spawn at different positions
3. **sjtu_drone.sdf** - Remove hardcoded namespace pollution
4. **drone_object.py** - Support namespace parameter for topic isolation
5. **plugin_drone.cpp** - Fix physics lookup to use model-scoped links

---

## Detailed Changes

### 1. Spawn Script Enhancement (`spawn_drone.py`)

**File**: `src/sjtu_drone/sjtu_drone_bringup/sjtu_drone_bringup/spawn_drone.py`

**Purpose**: Allow spawning drones at different 3D positions.

**Changes Made**:
```python
# Added position argument parsing (lines ~30-35)
x, y, z = 0.0, 0.0, 0.3  # Default position
if len(sys.argv) >= 6:
    x = float(sys.argv[3])
    y = float(sys.argv[4])
    z = float(sys.argv[5])

# Added file path detection (lines ~20-25)
if os.path.isfile(robot_model):
    with open(robot_model, 'r') as f:
        robot_model = f.read()
```

**Impact**: Enables spawning drones at different coordinates via command-line arguments.

---

### 2. Launch File Modifications (`drone_dual_tb3_gazebo.launch.py`)

**File**: `src/drive_drone/launch/drone_dual_tb3_gazebo.launch.py`

**Purpose**: Process xacro files with collision/physics data and spawn drones at different positions.

**Changes Made**:

```python
# Import tempfile module
import tempfile

# Process xacro to include collision and physics (lines ~40-60)
drone1_doc = xacro.process_file(
    drone_xacro_path,
    mappings={
        'collision_enabled': 'true',
        'physics_enabled': 'true'
    }
)

# Save to temporary file
drone1_temp = tempfile.NamedTemporaryFile(
    mode='w', 
    suffix='.urdf', 
    delete=False
)
drone1_temp.write(drone1_doc.toxml())
drone1_temp.close()

# Spawn with position arguments
Node(
    package='sjtu_drone_bringup',
    executable='spawn_drone',
    arguments=[
        'drone1',
        drone1_temp.name,  # Pass file path
        '0.0', '0.0', '0.3'  # Position (x, y, z)
    ],
    # ...
)

# Drone2 at different position
Node(
    # ...
    arguments=[
        'drone2',
        drone2_temp.name,
        '3.0', '0.0', '0.3'  # 3 meters offset on X-axis
    ],
    # ...
)
```

**Impact**: 
- Drones spawn with full collision/physics data
- Drone1 at (0, 0, 0.3), Drone2 at (3, 0, 0.3)

---

### 3. SDF Namespace Cleanup (`sjtu_drone.sdf`)

**File**: `src/sjtu_drone/sjtu_drone_description/models/sjtu_drone/sjtu_drone.sdf`

**Purpose**: Remove hardcoded namespaces to allow dynamic namespace assignment.

**Changes Made**:

Removed `<namespace>/simple_drone</namespace>` from 6 plugin blocks:

```xml
<!-- BEFORE: Hardcoded namespace -->
<plugin name='imu' filename='libgazebo_ros_imu_sensor.so'>
  <ros>
    <namespace>/simple_drone</namespace>
  </ros>
  <!-- ... -->
</plugin>

<!-- AFTER: Empty <ros> tag inherits namespace from spawn request -->
<plugin name='imu' filename='libgazebo_ros_imu_sensor.so'>
  <ros>
  </ros>
  <!-- ... -->
</plugin>
```

**Affected Plugins** (all 6 cleaned):
- `imu` (libgazebo_ros_imu_sensor.so)
- `gps` (libgazebo_ros_gps_sensor.so)
- `camera_bottom` (libgazebo_ros_camera.so)
- `camera_front` (libgazebo_ros_camera.so)
- `sonar` (libgazebo_ros_ray_sensor.so)
- `simple_drone` (libplugin_drone.so) - Main physics plugin

**Impact**: Each drone instance now uses its own namespace (`/drone1/*`, `/drone2/*`) instead of all sharing `/simple_drone/*`.

---

### 4. DroneObject Namespace Support (`drone_object.py`)

**File**: `src/sjtu_drone/sjtu_drone_control/sjtu_drone_control/drone_utils/drone_object.py`

**Purpose**: Allow Python control API to target specific drones by namespace.

**Changes Made**:

```python
# Modified __init__ to accept namespace parameter
def __init__(self, namespace=''):
    super().__init__('drone_object')
    self.namespace = namespace
    
    # Use relative topic paths (not private ~/topics)
    # BEFORE: self.posepub = self.create_publisher(Pose, '~/gt_pose', 1)
    # AFTER:
    self.posepub = self.create_publisher(Pose, 'gt_pose', 1)
    
    self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
    self.takeoff_pub = self.create_publisher(Empty, 'takeoff', 1)
    self.land_pub = self.create_publisher(Empty, 'land', 1)
    self.reset_pub = self.create_publisher(Empty, 'reset', 1)
    self.posctrl_pub = self.create_publisher(Bool, 'posctrl', 1)
    
    # Subscribers updated similarly (13 topics total)
    self.imusub = self.create_subscription(Imu, 'imu', self.ImuCallback, 1)
    # ... and 6 more subscribers
```

**Usage Example**:
```python
# Control specific drone
drone1 = DroneObject(namespace='drone1')
drone2 = DroneObject(namespace='drone2')

drone1.takeoff()  # Only drone1 takes off
drone2.takeoff()  # Only drone2 takes off
```

**Impact**: Python API can now control individual drones by specifying namespace.

---

### 5. Physics Plugin Fix (`plugin_drone.cpp`) ⭐ **CRITICAL**

**File**: `src/sjtu_drone/sjtu_drone_description/src/plugin_drone.cpp`

**Purpose**: Fix physics hijacking bug where drone2's plugin controlled drone1's body.

**Root Cause**: 
The plugin searched the entire Gazebo world for a link named `"base_footprint"`. Since both drones have identically named links, drone2's plugin would find and control drone1's body.

**Changes Made**:

```cpp
// Line 73 - BEFORE: Global world search
link = world->EntityByName(link_name_);

// Line 73 - AFTER: Model-scoped link lookup
link = _model->GetLink(link_name_);
```

**Technical Details**:
- `world->EntityByName()` searches ALL entities in the simulation world
- `_model->GetLink()` searches only within the current model instance
- This ensures each plugin instance controls only its own drone's physics

**Impact**: 
- Each drone's physics plugin now operates independently
- Commands to `/drone2/cmd_vel` only affect drone2's body
- Commands to `/drone1/takeoff` only affect drone1's body

**Build Requirement**: This is a C++ change requiring recompilation:
```bash
colcon build --packages-select sjtu_drone_description sjtu_drone_control
```

---

## Testing Independent Control

After building and launching, test namespace isolation:

### Test 1: Independent Takeoff
```bash
# Terminal 1 - Drone1 should take off
ros2 topic pub --once /drone1/takeoff std_msgs/msg/Empty

# Terminal 2 - Drone2 should take off (drone1 unaffected)
ros2 topic pub --once /drone2/takeoff std_msgs/msg/Empty
```

### Test 2: Independent Movement
```bash
# Move drone1 forward
ros2 topic pub --once /drone1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 1.0, y: 0.0, z: 0.0}}'

# Move drone2 backward (drone1 continues forward)
ros2 topic pub --once /drone2/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: -1.0, y: 0.0, z: 0.0}}'
```

### Test 3: List Topics
```bash
# Should show separate namespaces
ros2 topic list | grep -E "drone[12]"
```

Expected output:
```
/drone1/camera_bottom/camera_info
/drone1/camera_bottom/image_raw
/drone1/cmd_vel
/drone1/imu
/drone1/takeoff
/drone1/land
/drone1/reset
...
/drone2/camera_bottom/camera_info
/drone2/camera_bottom/image_raw
/drone2/cmd_vel
/drone2/imu
/drone2/takeoff
...
```

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    Gazebo World                              │
│                                                              │
│  ┌─────────────────────┐      ┌─────────────────────┐      │
│  │   Drone1 Model      │      │   Drone2 Model      │      │
│  │   @ (0, 0, 0.3)     │      │   @ (3, 0, 0.3)     │      │
│  ├─────────────────────┤      ├─────────────────────┤      │
│  │ libplugin_drone.so  │      │ libplugin_drone.so  │      │
│  │ Controls: drone1's  │      │ Controls: drone2's  │      │
│  │ base_footprint ONLY │      │ base_footprint ONLY │      │
│  └─────────────────────┘      └─────────────────────┘      │
│           │                             │                   │
└───────────┼─────────────────────────────┼───────────────────┘
            │                             │
            ▼                             ▼
     ┌─────────────┐              ┌─────────────┐
     │ /drone1/*   │              │ /drone2/*   │
     │  - cmd_vel  │              │  - cmd_vel  │
     │  - takeoff  │              │  - takeoff  │
     │  - imu      │              │  - imu      │
     │  - camera_* │              │  - camera_* │
     └─────────────┘              └─────────────┘
```

---

## Key Lessons Learned

### Multi-Robot Gazebo Best Practices

1. **Namespace Cleanliness**: SDF files must not hardcode namespaces - use empty `<ros></ros>` tags
2. **Relative Topic Names**: Use relative paths (`cmd_vel`) not private paths (`~/cmd_vel`) in SDF
3. **Model-Scoped Physics**: C++ plugins must use `_model->GetLink()` not `world->EntityByName()`
4. **URDF Processing**: Always process xacro files with collision/physics parameters
5. **Position Separation**: Spawn at different positions to prevent initial overlap

---

## File Summary

| File | Lines Changed | Purpose |
|------|--------------|---------|
| `spawn_drone.py` | ~15 | Position arguments + file path support |
| `drone_dual_tb3_gazebo.launch.py` | ~50 | Xacro processing + tempfile + position spawning |
| `sjtu_drone.sdf` | 6 blocks | Remove hardcoded `/simple_drone` namespace |
| `drone_object.py` | ~13 topics | Add namespace parameter + relative paths |
| `plugin_drone.cpp` | 1 line | **CRITICAL**: Model-scoped link lookup |

---

## Troubleshooting

### Both drones move together
- **Cause**: plugin_drone.cpp not rebuilt with model-scoped fix
- **Solution**: `colcon build --packages-select sjtu_drone_description`

### Drone falls through ground
- **Cause**: xacro not processed with collision/physics
- **Solution**: Verify launch file uses `xacro.process_file()` with proper mappings

### Topics appear under wrong namespace
- **Cause**: Hardcoded namespace in SDF or private `~/` topic paths
- **Solution**: Check sjtu_drone.sdf has empty `<ros></ros>` tags only

### Build fails with gazebo_ros not found
- **Cause**: Conda environment interfering with ROS packages
- **Solution**: `conda deactivate` before building

---

## Future Enhancements

- [ ] Dynamic drone count (N drones via launch parameter)
- [ ] Collision avoidance between drones
- [ ] Formation flying controller
- [ ] Multi-drone SLAM integration
- [ ] Centralized fleet manager node

---

## Credits

**Modified by**: GitHub Copilot (Claude Sonnet 4.5)  
**Date**: January 31, 2026  
**Original Package**: SJTU Drone (ros-gazebo-gym)  
**ROS Version**: ROS 2 Humble  
**Gazebo Version**: 11.10.2
