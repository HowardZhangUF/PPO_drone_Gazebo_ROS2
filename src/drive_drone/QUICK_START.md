# Quick Start Guide: Dual TB3 Road Network

## 🚀 Quick Start (3 Steps)

### Step 1: Build
```bash
cd ~/ros2_ws
colcon build --packages-select drive_drone
source install/setup.bash
```

### Step 2: Set TurtleBot3 Model
```bash
export TURTLEBOT3_MODEL=burger
```

### Step 3: Launch
```bash
# Terminal 1: Start Gazebo (empty world)
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Terminal 2: Launch dual TB3 system (wait for Gazebo to load)
ros2 launch drive_drone dual_tb3_road_network.launch.py
```

## 📊 Monitor

### Check Nodes
```bash
ros2 node list
# Expected:
#   /tb3_1/road_network_mover
#   /tb3_2/road_network_mover
```

### Check Topics
```bash
ros2 topic list | grep -E "(tb3_1|tb3_2)"
# Expected:
#   /tb3_1/cmd_vel
#   /tb3_1/odom
#   /tb3_1/current_goal
#   /tb3_2/cmd_vel
#   /tb3_2/odom
#   /tb3_2/current_goal
```

### Watch Robot 1
```bash
ros2 topic echo /tb3_1/cmd_vel
```

### Watch Robot 2
```bash
ros2 topic echo /tb3_2/cmd_vel
```

## 🎯 Key Concepts

### Namespaces
- **TB3_1**: `/tb3_1/*` - First robot (starts at Node 0: -1.5, 1.0)
- **TB3_2**: `/tb3_2/*` - Second robot (starts at Node 5: 0.3, 0.0)

### Topic Names
| Robot | Velocity | Odometry | Goal |
|-------|----------|----------|------|
| TB3_1 | `/tb3_1/cmd_vel` | `/tb3_1/odom` | `/tb3_1/current_goal` |
| TB3_2 | `/tb3_2/cmd_vel` | `/tb3_2/odom` | `/tb3_2/current_goal` |

### Road Network (8 nodes)
```
     N1 ----→ N0        N2
     ↓        ↓         ↑
     ↓        ↓         ↑
     N3       N4 → N5 → N6
              ↑
              N7
```

## 🔧 Customization

### Change Starting Positions
Edit `launch/dual_tb3_road_network.launch.py`:
```python
# Robot 1 position
'x_pose': '-1.5',  # Change X
'y_pose': '1.0',   # Change Y

# Robot 2 position  
'x_pose': '0.3',   # Change X
'y_pose': '0.0',   # Change Y
```

### Add Third Robot
1. Copy TB3_2 section in launch file
2. Change namespace to `tb3_3`
3. Set different starting position
4. Rebuild: `colcon build --packages-select drive_drone`

## ❗ Troubleshooting

| Problem | Solution |
|---------|----------|
| Robots don't spawn | Set `export TURTLEBOT3_MODEL=burger` |
| Robots don't move | Check if mover nodes are running: `ros2 node list` |
| Topics missing | Verify namespace in launch file |
| Build fails | Check [tb3_road_network_mover.cpp](src/tb3_road_network_mover.cpp) syntax |

## 📁 Files Modified/Created

### Modified
- ✏️ `src/tb3_road_network_mover.cpp` - Added namespace support

### Created
- ✨ `launch/dual_tb3_road_network.launch.py` - Launch file
- 📖 `DUAL_TB3_README.md` - Full documentation
- 🎨 `visualize_dual_tb3.py` - Visualization script
- 📋 `QUICK_START.md` - This file

## 📚 More Info

See [DUAL_TB3_README.md](DUAL_TB3_README.md) for complete documentation.

## 🎓 Learning Resources

### Why Namespaces?
Without namespaces, both robots would publish to the same `/cmd_vel` topic, causing conflicts. With namespaces, each robot has its own isolated topic space.

### Absolute vs Relative Topics
```cpp
// ❌ Absolute (ignores namespace)
create_publisher<Twist>("/cmd_vel", 10);

// ✅ Relative (respects namespace)  
create_publisher<Twist>("cmd_vel", 10);
```

### How It Works
1. Gazebo spawns two TB3 robots
2. Each robot has a namespace (`/tb3_1`, `/tb3_2`)
3. Road network mover nodes subscribe to `odom` (becomes `/tb3_1/odom`, `/tb3_2/odom`)
4. Nodes publish to `cmd_vel` (becomes `/tb3_1/cmd_vel`, `/tb3_2/cmd_vel`)
5. Each robot independently navigates the road network using Markov chains

---
**Happy Robot Navigation! 🤖🤖**
