#!/usr/bin/env python3
# filepath: /home/basestation/ros2_ws/src/drive_drone/launch/drone_dual_tb3_gazebo.launch.py

"""
Launch file for TWO drones + dual TurtleBot3 robots with road network navigation.
FIXED: Process xacro and save to temp file for proper collision/physics
"""

import os
import yaml
import tempfile
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
import xacro

def generate_launch_description():

    drone_pkg = get_package_share_directory('sjtu_drone_bringup')
    drone_description_pkg = get_package_share_directory('sjtu_drone_description')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    model_folder = 'turtlebot3_burger'
    urdf_path = os.path.join(turtlebot3_gazebo_dir, 'models', model_folder, 'model.sdf')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    use_gui = DeclareLaunchArgument("use_gui", default_value="true", choices=["true", "false"],
                                    description="Whether to execute gzclient")

    # ═══════════════════════════════════════════════════════
    # 1. GAZEBO SERVER AND CLIENT
    # ═══════════════════════════════════════════════════════
    world_file_default = os.path.join(drone_description_pkg, "worlds", "playground.world")
    world_file = LaunchConfiguration('world', default=world_file_default)
    
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=world_file_default,
        description='Full path to world file to load'
    )
    
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file,
                          'verbose': "true",
                          'extra_gazebo_args': 'verbose'}.items()
    )
    
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items()
    )

    # ═══════════════════════════════════════════════════════
    # 2. DRONE SETUP (TWO DRONES)
    # ═══════════════════════════════════════════════════════
    xacro_file_name = "sjtu_drone.urdf.xacro"
    xacro_file = os.path.join(drone_description_pkg, "urdf", xacro_file_name)
    yaml_file_path = os.path.join(drone_pkg, 'config', 'drone.yaml')
    
    # Process xacro to get full URDF with collision/physics
    robot_description_config = xacro.process_file(xacro_file, mappings={"params_path": yaml_file_path})
    robot_desc = robot_description_config.toxml()
    
    # Save processed URDF to temporary file for spawn_drone script
    temp_urdf = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf', prefix='drone_')
    temp_urdf.write(robot_desc)
    temp_urdf.flush()
    urdf_file_path = temp_urdf.name
    temp_urdf.close()
    
    # Drone namespaces
    drone1_ns = "drone1"
    drone2_ns = "drone2"

    # Robot State Publishers for Drone 1
    robot_state_publisher_drone1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=drone1_ns,
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time, 
            "robot_description": robot_desc, 
            "frame_prefix": drone1_ns + "/"
        }],
        arguments=[robot_desc]
    )
    
    joint_state_publisher_drone1 = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=drone1_ns,
        output='screen',
    )
    
    # Robot State Publishers for Drone 2
    robot_state_publisher_drone2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=drone2_ns,
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time, 
            "robot_description": robot_desc, 
            "frame_prefix": drone2_ns + "/"
        }],
        arguments=[robot_desc]
    )
    
    joint_state_publisher_drone2 = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=drone2_ns,
        output='screen',
    )
    
    # Static TF for Drone 1
    static_tf_drone1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", f"{drone1_ns}/odom"],
        output="screen"
    )
    
    # Static TF for Drone 2
    static_tf_drone2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", f"{drone2_ns}/odom"],
        output="screen"
    )
    
    # Spawn Drone 1 at position (0, 0, 0.3) using processed URDF from temp file
    spawn_drone1 = Node(
        package="sjtu_drone_bringup",
        executable="spawn_drone",
        arguments=[urdf_file_path, drone1_ns, "0.0", "0.0", "0.3"],
        output="screen"
    )
    
    spawn_drone1_delayed = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to be ready
        actions=[spawn_drone1]
    )
    
    # Spawn Drone 2 at position (3.0, 0, 0.3) - offset from drone 1
    spawn_drone2_node = Node(
        package="sjtu_drone_bringup",
        executable="spawn_drone",
        arguments=[urdf_file_path, drone2_ns, "3.0", "0.0", "0.3"],
        output="screen"
    )
    
    spawn_drone2_delayed = TimerAction(
        period=7.0,  # Stagger by 2 seconds after drone1
        actions=[spawn_drone2_node]
    )

    # ═══════════════════════════════════════════════════════
    # 3. SPAWN TB3 ROBOTS
    # ═══════════════════════════════════════════════════════
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
        period=10.0,  # Wait for drones to spawn first
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
        period=12.0,  # Stagger spawns by 2 seconds
        actions=[spawn_tb3_2]
    )

    # ═══════════════════════════════════════════════════════
    # 4. ROAD NETWORK MOVERS (start after spawning)
    # ═══════════════════════════════════════════════════════
    tb3_1_mover = Node(
        package='drive_drone',
        executable='tb3_road_network_mover',
        name='road_network_mover',
        namespace='tb3_1',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_namespace': 'tb3_1',
            'start_node': 0
        }],
        output='screen',
        respawn=True,
    )
    
    tb3_1_mover_delayed = TimerAction(
        period=15.0,  # Start after spawning completes
        actions=[tb3_1_mover]
    )

    tb3_2_mover = Node(
        package='drive_drone',
        executable='tb3_road_network_mover',
        name='road_network_mover',
        namespace='tb3_2',
        parameters=[{
            'use_sim_time': use_sim_time,
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
        use_gui,
        world_arg,
        
        # Step 1: Launch Gazebo (t=0)
        gzserver,
        gzclient,
        
        # Step 2: Start drone robot state publishers immediately (t=0)
        robot_state_publisher_drone1,
        joint_state_publisher_drone1,
        static_tf_drone1,
        
        robot_state_publisher_drone2,
        joint_state_publisher_drone2,
        static_tf_drone2,
        
        # Step 3: Spawn drones (t=5s, t=7s)
        spawn_drone1_delayed,
        spawn_drone2_delayed,
        
        # Step 4: Spawn TB3 robots (t=10s, t=12s)
        spawn_tb3_1_delayed,
        spawn_tb3_2_delayed,
        
        # Step 5: Start road network movers (t=15s)
        tb3_1_mover_delayed,
        tb3_2_mover_delayed,
    ])
