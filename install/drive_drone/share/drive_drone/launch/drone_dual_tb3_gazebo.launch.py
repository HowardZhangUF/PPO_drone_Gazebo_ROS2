#!/usr/bin/env python3
# filepath: /home/basestation/ros2_ws/src/drive_drone/launch/drone_dual_tb3_gazebo.launch.py

"""
Launch file for drone + dual TurtleBot3 robots with road network navigation.
FIXED: Adds proper delays for Gazebo service availability
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    DeclareLaunchArgument,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    drone_pkg = get_package_share_directory('sjtu_drone_bringup')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    model_folder = 'turtlebot3_burger'
    urdf_path = os.path.join(turtlebot3_gazebo_dir, 'models', model_folder, 'model.sdf')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # ═══════════════════════════════════════════════════════
    # 1. DRONE BRINGUP (includes Gazebo)
    # ═══════════════════════════════════════════════════════
    drone_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(drone_pkg, 'launch', 'sjtu_drone_gazebo.launch.py')
        ),
    )

    # ═══════════════════════════════════════════════════════
    # 2. WAIT FOR GAZEBO, THEN SPAWN TB3 ROBOTS
    # ═══════════════════════════════════════════════════════
    # ✅ FIX: Add 10 second delay to let Gazebo fully start
    
    # TB3_1 Spawning
    spawn_tb3_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='tb3_1',
        arguments=[
            '-entity', 'tb3_1',
            '-file', urdf_path,
            '-x', '-1.5',
            '-y', '1.0',
            '-z', '0.01',
            '-robot_namespace', 'tb3_1'
        ],
        output='screen',
    )
    
    spawn_tb3_1_delayed = TimerAction(
        period=10.0,  # ✅ Wait 10 seconds for Gazebo
        actions=[spawn_tb3_1]
    )

    # TB3_2 Spawning
    spawn_tb3_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='tb3_2',
        arguments=[
            '-entity', 'tb3_2',
            '-file', urdf_path,
            '-x', '0.3',
            '-y', '0.0',
            '-z', '0.01',
            '-robot_namespace', 'tb3_2'
        ],
        output='screen',
    )
    
    spawn_tb3_2_delayed = TimerAction(
        period=12.0,  # ✅ Stagger spawns by 2 seconds
        actions=[spawn_tb3_2]
    )

    # ═══════════════════════════════════════════════════════
    # 3. ROAD NETWORK MOVERS (start after spawning)
    # ═══════════════════════════════════════════════════════
    tb3_1_mover = Node(
        package='drive_drone',
        executable='tb3_road_network_mover',
        name='road_network_mover',
        namespace='tb3_1',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_namespace': 'tb3_1',
            'start_node': 0
        }],
        output='screen',
        respawn=True,
    )
    
    tb3_1_mover_delayed = TimerAction(
        period=15.0,  # ✅ Start after spawning completes
        actions=[tb3_1_mover]
    )

    tb3_2_mover = Node(
        package='drive_drone',
        executable='tb3_road_network_mover',
        name='road_network_mover',
        namespace='tb3_2',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_namespace': 'tb3_2',
            'start_node': 5
        }],
        output='screen',
        respawn=True,
    )
    
    tb3_2_mover_delayed = TimerAction(
        period=15.0,
        actions=[tb3_2_mover]
    )

    # ═══════════════════════════════════════════════════════
    # LAUNCH SEQUENCE WITH PROPER TIMING
    # ═══════════════════════════════════════════════════════
    return LaunchDescription([
        use_sim_time_arg,
        
        # Step 1: Launch Gazebo with drone (t=0)
        drone_bringup,
        
        # Step 2: Spawn TB3 robots (t=10s, t=12s)
        spawn_tb3_1_delayed,
        spawn_tb3_2_delayed,
        
        # Step 3: Start road network movers (t=15s)
        tb3_1_mover_delayed,
        tb3_2_mover_delayed,
    ])