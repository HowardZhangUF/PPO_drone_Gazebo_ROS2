# Gazebo Robot Spawning Issue - Debug Documentation

**Date**: January 27, 2026  
**Status**: 🔴 UNRESOLVED - Robots not spawning in Gazebo environment

---

## Executive Summary

A previously working multi-robot simulation (SJTU drone UAV + dual TurtleBot3 robots) has stopped spawning robots in Gazebo. The environment loads successfully, but no robots appear. The RL-based tracking system cannot function without spawned robots.

---

## Previously Working Configuration

### Successful Launch Sequence (Historical)

Terminal 1 - Launch Simulation:
```bash
cd ~/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch drive_drone drone_dual_tb3_gazebo.launch.py
```

Terminal 2 - Launch RL Multi-Target Tracker:
```bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run mml_guidance rl_gazebo_multi_tracker \
  --ros-args \
  -p model_path:=/home/basestation/ros2_ws/src/mml_python_sim/models_recurrent/recurrent_ppo_tracking_final_20251216_151239.zip \
  -p max_vel:=0.8 \
  -p drone_height:=3.0 \
  -p target1_topic:=/tb3_1/odom \
  -p target2_topic:=/tb3_2/odom
```

### Expected Behavior (What Used to Work)
- ✅ Gazebo opens with `playground.world` environment
- ✅ SJTU drone UAV spawns at initial position
- ✅ Two TurtleBot3 'burger' robots spawn at their designated positions
- ✅ TurtleBot3 robots navigate road network using Markov chain logic
- ✅ UAV takes off and tracks TurtleBot3 targets using trained RL policy
- ✅ All odometry topics publish correctly (`/tb3_1/odom`, `/tb3_2/odom`, `/drone/odom`)

---

## Current Problem - Detailed Symptoms

### What Happens Now (Broken Behavior)
- ✅ Gazebo window opens successfully
- ✅ `playground.world` environment loads (walls, construction elements, actors visible)
- ❌ **SJTU drone UAV does NOT spawn**
- ❌ **TurtleBot3 robot #1 does NOT spawn**
- ❌ **TurtleBot3 robot #2 does NOT spawn**
- ❌ Empty environment - no robots visible
- ❌ RL tracker cannot function (no target odometry topics available)

### Error Messages Observed

#### Spawn Service Unavailable Error
```
[INFO] [spawn_entity.py]: Waiting for service /spawn_entity, timeout = 30
[ERROR] [spawn_entity.py]: Service /spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?
```

#### Custom Spawn Script Issues
- SJTU drone package includes custom `spawn_drone` script
- Custom script incompatible with standard Gazebo ROS 2 workflow
- Script expects different service interface than `gazebo_ros` provides

---

## System Configuration

### ROS 2 Environment
- **ROS Distribution**: Humble (Ubuntu 22.04)
- **Middleware**: rmw_fastrtps_cpp
- **Workspace**: `/home/basestation/ros2_ws`
- **Build System**: colcon

### Gazebo Configuration
- **Gazebo Version**: 11.10.2
- **Gazebo ROS**: gazebo_ros_pkgs for ROS 2 Humble
- **Model Path**: 
  ```
  GAZEBO_MODEL_PATH=:/home/basestation/ros2_ws/install/sjtu_drone_description/share/sjtu_drone_description/models:/opt/ros/humble/share/turtlebot3_gazebo/models
  ```

### Package Inventory
**SJTU Drone Packages** (3 total):
- `sjtu_drone_bringup` - Launch files
- `sjtu_drone_control` - Control systems
- `sjtu_drone_description` - URDF, meshes, worlds, Gazebo plugin

**TurtleBot3 Packages** (40+ packages installed):
- `turtlebot3_gazebo` - Gazebo integration
- `turtlebot3_description` - Robot models
- All standard TurtleBot3 support packages

**Gazebo ROS Packages** (16 packages):
- `gazebo_ros_pkgs`
- `gazebo_ros`
- `gazebo_plugins`
- All required integration packages

**Custom Packages**:
- `drive_drone` - Contains `drone_dual_tb3_gazebo.launch.py` and TB3 navigation logic
- `mml_guidance` - RL-based tracking node
- `mml_python_sim` - Simulation utilities and trained models

---

## Diagnostic Steps Performed

### 1. Gazebo Installation Verification ✅
```bash
$ gazebo --version
Gazebo multi-robot simulator, version 11.10.2
```
**Result**: Gazebo correctly installed

### 2. ROS 2 Package Verification ✅
```bash
$ ros2 pkg list | grep -E "gazebo|sjtu|turtlebot3"
```
**Result**: All 16 gazebo packages, 3 sjtu_drone packages, and 40+ turtlebot3 packages present

### 3. Plugin Compilation Check ✅
```bash
$ ls ~/ros2_ws/install/sjtu_drone_description/lib/libplugin_drone.so
/home/basestation/ros2_ws/install/sjtu_drone_description/lib/libplugin_drone.so
```
**Result**: SJTU drone Gazebo plugin successfully compiled

### 4. URDF File Verification ✅
```bash
$ ls ~/ros2_ws/install/sjtu_drone_description/share/sjtu_drone_description/urdf/sjtu_drone.urdf
```
**Result**: URDF file exists (SJTU drone uses URDF, not SDF)

### 5. Service Availability Check ❌
```bash
$ ros2 service list | grep spawn
# Returns empty - no spawn service available
```
**Result**: `/spawn_entity` service NOT available (Gazebo not running or not configured)

### 6. Running Nodes Check
```bash
$ ros2 node list
# Returns empty when simulation not running
```

### 7. Environment Variable Verification ✅
```bash
$ printenv | grep GAZEBO
GAZEBO_MODEL_PATH=:/home/basestation/ros2_ws/install/sjtu_drone_description/share/sjtu_drone_description/models:/opt/ros/humble/share/turtlebot3_gazebo/models
```
**Result**: Model paths correctly configured

### 8. Build System Verification ✅
```bash
$ colcon build --packages-select sjtu_drone_bringup sjtu_drone_control sjtu_drone_description
# Build successful with warnings (CMAKE policy-related, non-critical)
```

---

## Technical Analysis

### Root Cause Investigation

#### Issue #1: Spawn Service Timing Problem
- **Finding**: `/spawn_entity` service not available when spawn attempts occur
- **Hypothesis**: Gazebo takes time to start and initialize gazebo_ros plugins
- **Impact**: spawn_entity.py times out after 30 seconds waiting for service
- **Why It's Critical**: Without spawn service, cannot load robot URDF/SDF into simulation

#### Issue #2: Custom Spawn Script Incompatibility
- **Finding**: SJTU drone package uses custom `spawn_drone` script
- **File Location**: Likely in `sjtu_drone_bringup/scripts/`
- **Problem**: Custom script expects different service interface
- **Standard Method**: Should use `gazebo_ros/spawn_entity.py` with `-entity`, `-file`, `-x`, `-y`, `-z` parameters
- **Workaround Needed**: Must modify launch file to use standard spawning

#### Issue #3: Launch File Architecture
**File**: [src/drive_drone/launch/drone_dual_tb3_gazebo.launch.py](src/drive_drone/launch/drone_dual_tb3_gazebo.launch.py)

**Current Approach**:
1. Launches Gazebo with world file
2. Attempts to spawn drone using custom mechanism
3. Spawns two TurtleBot3 robots using spawn_entity.py
4. Launches navigation nodes

**Suspected Problems**:
- Insufficient delay between Gazebo start and spawn attempts
- Custom drone spawn incompatible with gazebo_ros
- Missing LaunchConfiguration options
- Incorrect service name/namespace

#### Issue #4: URDF vs SDF Confusion
- **Finding**: SJTU drone uses URDF format, not model.sdf
- **Impact**: Looking for model.sdf causes confusion
- **Solution**: Must use `-file` parameter with URDF path when spawning
- **Plugin**: libplugin_drone.so must be specified in URDF `<gazebo>` tags

### World File Configuration
**File**: [src/sjtu_drone/sjtu_drone_description/worlds/playground.world](src/sjtu_drone/sjtu_drone_description/worlds/playground.world)

**Contents**:
- Complex environment with construction theme
- Moving actors (pedestrians)
- Static obstacles (walls, cones, dumpsters)
- Physics engine configuration
- Plugin configurations

**Potential Issues**:
- Missing model dependencies
- Plugin loading failures
- Physics parameter conflicts
- Gazebo version incompatibilities

---

## Comparison: What Changed?

### Hypothesis: System State Differences

**Possible Changes That Could Cause This Issue**:

1. **ROS 2 Package Updates**
   - Gazebo ROS packages updated to incompatible version
   - TurtleBot3 packages updated with breaking changes
   - Check: `apt list --installed | grep ros-humble-gazebo`

2. **Gazebo Version Change**
   - Gazebo upgraded from older version to 11.10.2
   - Plugin API changes
   - Check: Review system update history

3. **Environment Variable Changes**
   - `GAZEBO_MODEL_PATH` modified or overwritten
   - `.bashrc` or workspace setup altered
   - Check: Compare current vs backup dotfiles

4. **Launch File Modifications**
   - `drone_dual_tb3_gazebo.launch.py` edited since last working state
   - Timing parameters changed
   - Check: `git diff` or version control history

5. **Build System State**
   - Incomplete rebuild after source changes
   - Cached CMake configurations stale
   - Check: Clean rebuild all packages

6. **Plugin Path Issues**
   - `libplugin_drone.so` not found at runtime
   - `GAZEBO_PLUGIN_PATH` not set correctly
   - Check: `printenv | grep GAZEBO_PLUGIN_PATH`

---

## Troubleshooting Roadmap

### Phase 1: Isolate the Problem

#### Test 1.1: Basic SJTU Drone Launch
**Purpose**: Verify SJTU drone simulator works independently

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch sjtu_drone_bringup sjtu_drone_gazebo.launch.py
```

**Success Criteria**:
- Gazebo opens with playground world
- Drone spawns and is visible
- No spawn service errors

**If Fails**: SJTU drone package has fundamental issue → Investigate upstream repository

#### Test 1.2: Gazebo World File Direct Load
**Purpose**: Verify world file loads without spawn issues

```bash
gazebo ~/ros2_ws/install/sjtu_drone_description/share/sjtu_drone_description/worlds/playground.world
```

**Success Criteria**:
- Gazebo opens successfully
- All world elements visible (no missing model errors)
- No plugin loading errors

**If Fails**: World file has missing dependencies → Fix model references

#### Test 1.3: Manual Spawn with Standard Method
**Purpose**: Test if standard spawning mechanism works

```bash
# Terminal 1: Launch Gazebo with ROS integration
ros2 launch gazebo_ros gazebo.launch.py world:=$HOME/ros2_ws/install/sjtu_drone_description/share/sjtu_drone_description/worlds/playground.world

# Terminal 2: Wait for Gazebo to fully load, then verify service
ros2 service list | grep spawn_entity

# Terminal 3: Manually spawn drone
ros2 run gazebo_ros spawn_entity.py \
  -entity sjtu_drone \
  -file $HOME/ros2_ws/install/sjtu_drone_description/share/sjtu_drone_description/urdf/sjtu_drone.urdf \
  -x 0.0 -y 0.0 -z 0.5
```

**Success Criteria**:
- `/spawn_entity` service appears in service list
- Drone spawns successfully at specified position
- No timeout errors

**If Fails**: Gazebo ROS integration broken → Reinstall gazebo_ros_pkgs

### Phase 2: Fix Launch File

#### Test 2.1: Add Explicit Delays
**Modify**: `drone_dual_tb3_gazebo.launch.py`

**Changes Needed**:
```python
from launch.actions import TimerAction

# Add delay before spawning
spawn_with_delay = TimerAction(
    period=5.0,  # Wait 5 seconds after Gazebo starts
    actions=[spawn_drone_action]
)
```

#### Test 2.2: Replace Custom Spawn with Standard Method
**Remove**: Custom `spawn_drone` script calls

**Replace With**: Standard `spawn_entity.py` approach:
```python
spawn_drone = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'sjtu_drone',
        '-file', drone_urdf_path,
        '-x', '0.0',
        '-y', '0.0', 
        '-z', '0.5',
        '-Y', '0.0'
    ],
    output='screen'
)
```

#### Test 2.3: Add Service Availability Check
**Purpose**: Don't attempt spawn until service ready

```python
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

wait_for_spawn_service = ExecuteProcess(
    cmd=[[
        FindExecutable(name='ros2'),
        ' service list | grep -q spawn_entity && echo "Service ready" || sleep 1'
    ]],
    shell=True
)
```

### Phase 3: Environment Verification

#### Test 3.1: Check Plugin Path
```bash
echo $GAZEBO_PLUGIN_PATH
# Should include: /home/basestation/ros2_ws/install/sjtu_drone_description/lib
```

**If Empty**: Add to launch file or `.bashrc`:
```bash
export GAZEBO_PLUGIN_PATH=/home/basestation/ros2_ws/install/sjtu_drone_description/lib:$GAZEBO_PLUGIN_PATH
```

#### Test 3.2: Verify Resource Paths
```bash
# Check if all required files exist
ls ~/ros2_ws/install/sjtu_drone_description/share/sjtu_drone_description/urdf/sjtu_drone.urdf
ls ~/ros2_ws/install/sjtu_drone_description/share/sjtu_drone_description/worlds/playground.world
ls ~/ros2_ws/install/sjtu_drone_description/lib/libplugin_drone.so
ls ~/ros2_ws/install/turtlebot3_description/urdf/turtlebot3_burger.urdf
```

#### Test 3.3: Clean Rebuild
```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

### Phase 4: Alternative Launch Strategy

#### Test 4.1: Create Simplified Launch File
**Purpose**: Minimal working example without complexity

**New File**: `src/drive_drone/launch/simple_spawn_test.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

def generate_launch_description():
    # Paths
    world_file = os.path.join(
        os.environ['HOME'],
        'ros2_ws/install/sjtu_drone_description/share/sjtu_drone_description/worlds/playground.world'
    )
    
    drone_urdf = os.path.join(
        os.environ['HOME'],
        'ros2_ws/install/sjtu_drone_description/share/sjtu_drone_description/urdf/sjtu_drone.urdf'
    )
    
    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Spawn drone after delay
    spawn_drone = TimerAction(
        period=10.0,  # Wait 10 seconds
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'sjtu_drone',
                    '-file', drone_urdf,
                    '-x', '0', '-y', '0', '-z', '0.5'
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        gazebo,
        spawn_drone
    ])
```

**Test Command**:
```bash
ros2 launch drive_drone simple_spawn_test.launch.py
```

---

## Quick Reference Commands

### Check if Simulation is Running
```bash
ros2 node list
ros2 topic list
ros2 service list | grep spawn
```

### View Gazebo Process
```bash
ps aux | grep gazebo
```

### Kill Stuck Gazebo
```bash
killall -9 gazebo gzserver gzclient
```

### Monitor ROS 2 Logs
```bash
ros2 launch drive_drone drone_dual_tb3_gazebo.launch.py 2>&1 | tee launch_output.log
```

### Check TurtleBot3 Spawning
```bash
# Should see two spawn_entity processes if working correctly
ps aux | grep spawn_entity
```

---

## Related Files for Investigation

### Launch Files
- [src/drive_drone/launch/drone_dual_tb3_gazebo.launch.py](src/drive_drone/launch/drone_dual_tb3_gazebo.launch.py) - Main combined launch
- [src/drive_drone/launch/dual_tb3_road_network.launch.py](src/drive_drone/launch/dual_tb3_road_network.launch.py) - TB3 only
- `src/sjtu_drone/sjtu_drone_bringup/launch/sjtu_drone_gazebo.launch.py` - SJTU drone only

### Robot Descriptions
- [src/sjtu_drone/sjtu_drone_description/urdf/sjtu_drone.urdf](src/sjtu_drone/sjtu_drone_description/urdf/sjtu_drone.urdf) - Drone URDF
- `install/turtlebot3_description/share/turtlebot3_description/urdf/turtlebot3_burger.urdf` - TB3 URDF

### World Files
- [src/sjtu_drone/sjtu_drone_description/worlds/playground.world](src/sjtu_drone/sjtu_drone_description/worlds/playground.world)

### Plugin Source
- `src/sjtu_drone/sjtu_drone_description/src/plugin_drone.cpp` - Gazebo plugin implementation
- `build/sjtu_drone_description/libplugin_drone.so` - Compiled plugin

### Navigation Logic
- `src/drive_drone/src/tb3_road_network_mover.cpp` - TB3 Markov chain navigation

---

## Next Steps - Priority Order

### 🔥 Immediate Actions (Do First)
1. ✅ **Create this documentation** (COMPLETED)
2. ⏳ **Run Test 1.1**: Launch basic SJTU drone simulator alone
3. ⏳ **Run Test 1.2**: Load world file directly in Gazebo
4. ⏳ **Run Test 1.3**: Manual spawn test with standard method

### 🔧 If Basic Tests Pass
5. ⏳ **Examine** `drone_dual_tb3_gazebo.launch.py` for timing issues
6. ⏳ **Add delays** before spawn attempts (Test 2.1)
7. ⏳ **Replace custom spawn** with standard method (Test 2.2)

### 🔍 If Basic Tests Fail
8. ⏳ **Check git history** for recent changes to launch files
9. ⏳ **Verify GAZEBO_PLUGIN_PATH** includes plugin directory
10. ⏳ **Clean rebuild** entire workspace (Test 3.3)
11. ⏳ **Check system updates** that may have changed Gazebo/ROS versions

### 📋 Documentation Tasks
12. ⏳ **Compare** current launch file with last known working version
13. ⏳ **Document** exact error messages from each test
14. ⏳ **Record** Gazebo terminal output during spawn attempts
15. ⏳ **Create** simplified test launch file (Test 4.1)

---

## Success Criteria - When is This Resolved?

### ✅ Problem Considered SOLVED When:
1. Running `ros2 launch drive_drone drone_dual_tb3_gazebo.launch.py` successfully spawns all robots
2. SJTU drone UAV appears in Gazebo at expected position
3. Both TurtleBot3 robots (tb3_1 and tb3_2) spawn correctly
4. Odometry topics are published: `/tb3_1/odom`, `/tb3_2/odom`, `/drone/odom`
5. RL tracker node can subscribe to odometry and control drone
6. TurtleBot3 robots navigate road network using Markov chain logic
7. No spawn service timeout errors occur
8. Process is repeatable across multiple launches

---

## Additional Resources

### SJTU Drone Repository
- **GitHub**: https://github.com/NovoG93/sjtu_drone
- **Branch**: ros2
- **Documentation**: Check repository README for known issues

### TurtleBot3 Documentation
- **ROS 2 Humble Guide**: https://emanual.robotis.com/docs/en/platform/turtlebot3/
- **Gazebo Simulation**: Check official ROBOTIS documentation

### Gazebo ROS 2 Integration
- **Tutorial**: https://github.com/ros-simulation/gazebo_ros_pkgs/wiki
- **spawn_entity.py Documentation**: Check gazebo_ros package

### Debugging Resources
- **ROS 2 Launch System**: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html
- **Gazebo Plugins**: http://gazebosim.org/tutorials?cat=connect_ros

---

## Appendix: Full Launch File Review Needed

**TODO**: Detailed analysis of `drone_dual_tb3_gazebo.launch.py`
- Current spawn mechanism for drone
- Timing/ordering of entity spawns  
- Service dependency declarations
- Environment variable usage
- Node lifecycle management

**TODO**: Compare with working `sjtu_drone_gazebo.launch.py`
- Identify differences in spawn approach
- Extract working patterns
- Adapt for multi-robot scenario

---

## Change Log

| Date | Action | Result |
|------|--------|--------|
| 2026-01-27 | Created comprehensive debug documentation | Pending testing |
| 2026-01-27 | Performed system diagnostics | All packages installed correctly |
| 2026-01-27 | Identified spawn service timing issue | Root cause hypothesis formed |

---

**Last Updated**: January 27, 2026  
**Maintainer**: Research Team  
**Status**: 🔴 Issue reproduction documented, systematic debugging in progress
