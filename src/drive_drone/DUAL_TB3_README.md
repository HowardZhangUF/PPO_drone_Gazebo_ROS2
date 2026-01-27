# Dual TurtleBot3 Road Network Setup

This guide explains how to run two TurtleBot3 robots simultaneously in the same Gazebo environment, each navigating the road network with their own namespace.

## Architecture

### Namespaces
- **TB3 Robot 1**: `/tb3_1`
- **TB3 Robot 2**: `/tb3_2`

### Topics Structure

Each robot has its own isolated topic namespace:

**TB3 Robot 1:**
- `/tb3_1/cmd_vel` - Velocity commands
- `/tb3_1/odom` - Odometry data
- `/tb3_1/current_goal` - Current navigation goal
- `/tb3_1/scan` - Laser scan data

**TB3 Robot 2:**
- `/tb3_2/cmd_vel` - Velocity commands
- `/tb3_2/odom` - Odometry data
- `/tb3_2/current_goal` - Current navigation goal
- `/tb3_2/scan` - Laser scan data

### Road Network

Both robots navigate the same road network with 8 nodes:
```
Node 0: (-1.5,  1.0, 180°)
Node 1: ( 0.0,  1.0, 180°)
Node 2: ( 2.1,  1.0,  90°)
Node 3: (-1.5, -1.0, 270°)
Node 4: ( 0.0, -1.0,  90°)
Node 5: ( 0.3,  0.0,   0°)
Node 6: ( 1.7,  0.0,   0°)
Node 7: ( 2.1, -1.0, 270°)
```

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select drive_drone
source install/setup.bash
```

## Running

### Option 1: Dual TB3 Only

Launch two TurtleBot3 robots with road network movers:

```bash
# Terminal 1: Start Gazebo world (empty world)
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Terminal 2: Launch dual TB3 road network system
ros2 launch drive_drone dual_tb3_road_network.launch.py
```

### Option 2: Dual TB3 + Drone

Launch two TurtleBot3 robots and a drone:

```bash
# Terminal 1: Launch everything
ros2 launch drive_drone p1_b_tb3_drone_gazebo.launch.py  # (needs to be updated)
```

## Monitoring

### View All Topics

```bash
ros2 topic list
```

You should see topics for both robots:
```
/tb3_1/cmd_vel
/tb3_1/odom
/tb3_1/current_goal
/tb3_2/cmd_vel
/tb3_2/odom
/tb3_2/current_goal
...
```

### Monitor Individual Robots

**TB3 Robot 1:**
```bash
# Velocity commands
ros2 topic echo /tb3_1/cmd_vel

# Odometry
ros2 topic echo /tb3_1/odom

# Current goal
ros2 topic echo /tb3_1/current_goal
```

**TB3 Robot 2:**
```bash
# Velocity commands
ros2 topic echo /tb3_2/cmd_vel

# Odometry
ros2 topic echo /tb3_2/odom

# Current goal
ros2 topic echo /tb3_2/current_goal
```

### RViz Visualization

To visualize both robots simultaneously:

```bash
rviz2
```

Configure RViz:
1. Set Fixed Frame to `map` or `odom`
2. Add RobotModel displays:
   - Topic: `/tb3_1/robot_description`
   - Topic: `/tb3_2/robot_description`
3. Add LaserScan displays:
   - Topic: `/tb3_1/scan`
   - Topic: `/tb3_2/scan`
4. Add Pose displays for goals:
   - Topic: `/tb3_1/current_goal`
   - Topic: `/tb3_2/current_goal`

## Customization

### Changing Starting Positions

Edit [dual_tb3_road_network.launch.py](launch/dual_tb3_road_network.launch.py):

```python
# TB3 Robot 1
launch_arguments={
    'x_pose': '-1.5',  # X position (meters)
    'y_pose': '1.0',   # Y position (meters)
    'robot_name': 'tb3_1',
}

# TB3 Robot 2
launch_arguments={
    'x_pose': '0.3',   # X position (meters)
    'y_pose': '0.0',   # Y position (meters)
    'robot_name': 'tb3_2',
}
```

### Adding More Robots

To add a third robot (tb3_3):

1. Add to launch file:
```python
# TB3 Robot 3
tb3_3_bringup = GroupAction([
    PushRosNamespace('tb3_3'),
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_pkg, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': '2.1',
            'y_pose': '-1.0',
            'robot_name': 'tb3_3',
        }.items(),
    )
])

tb3_3_mover = Node(
    package='drive_drone',
    executable='tb3_road_network_mover',
    name='road_network_mover',
    namespace='tb3_3',
    parameters=[{
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'robot_namespace': 'tb3_3'
    }],
    output='screen',
    respawn=True,
)
```

2. Add to LaunchDescription:
```python
return LaunchDescription([
    use_sim_time_arg,
    tb3_1_bringup, tb3_1_mover,
    tb3_2_bringup, tb3_2_mover,
    tb3_3_bringup, tb3_3_mover,  # New robot
])
```

## Code Structure

### Modified Files

1. **[tb3_road_network_mover.cpp](src/tb3_road_network_mover.cpp)**
   - Added namespace parameter support
   - Changed from absolute topics (`/cmd_vel`) to relative topics (`cmd_vel`)
   - Topics automatically inherit namespace from node

2. **[dual_tb3_road_network.launch.py](launch/dual_tb3_road_network.launch.py)** (NEW)
   - Spawns two TB3 robots with different namespaces
   - Launches road network mover node for each robot
   - Sets starting positions for each robot

### Key Changes in C++ Node

```cpp
// Constructor - declare namespace parameter
this->declare_parameter<std::string>("robot_namespace", "");
robot_namespace_ = this->get_parameter("robot_namespace").as_string();

// Publishers/Subscribers use relative topics (no leading /)
cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, ...);
```

When node is launched in namespace `/tb3_1`, topics become:
- `cmd_vel` → `/tb3_1/cmd_vel`
- `odom` → `/tb3_1/odom`

## Troubleshooting

### Robots Not Spawning
- Check that Gazebo is running
- Verify turtlebot3_gazebo package is installed
- Ensure TURTLEBOT3_MODEL environment variable is set:
  ```bash
  export TURTLEBOT3_MODEL=burger  # or waffle, waffle_pi
  ```

### Robots Not Moving
- Check if road network mover nodes are running:
  ```bash
  ros2 node list | grep road_network
  ```
- Verify topics are published:
  ```bash
  ros2 topic hz /tb3_1/cmd_vel
  ros2 topic hz /tb3_2/cmd_vel
  ```

### Robots Colliding
- Adjust starting positions in launch file
- Modify road network to create separate lanes
- Add collision avoidance logic

### One Robot Works, Other Doesn't
- Check namespace configuration in launch file
- Verify both mover nodes are running
- Check for error messages in terminal

## Development Notes

### Why Use Namespaces?

Namespaces prevent topic name conflicts when running multiple robots:

**Without namespaces:**
- Both robots publish to `/cmd_vel`
- Both robots subscribe to `/odom`
- Commands get mixed, robots behave erratically

**With namespaces:**
- Robot 1: `/tb3_1/cmd_vel` and `/tb3_1/odom`
- Robot 2: `/tb3_2/cmd_vel` and `/tb3_2/odom`
- Each robot operates independently

### Relative vs Absolute Topics

**Absolute topics** (starting with `/`):
```cpp
// Always publishes to /cmd_vel regardless of namespace
create_publisher<Twist>("/cmd_vel", 10);
```

**Relative topics** (no leading `/`):
```cpp
// In namespace /tb3_1 → publishes to /tb3_1/cmd_vel
// In namespace /tb3_2 → publishes to /tb3_2/cmd_vel
create_publisher<Twist>("cmd_vel", 10);
```

## References

- TurtleBot3 Docs: https://emanual.robotis.com/docs/en/platform/turtlebot3/
- ROS 2 Namespaces: https://docs.ros.org/en/rolling/Concepts/About-ROS-2-Names.html
