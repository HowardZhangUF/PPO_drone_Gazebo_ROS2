#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    drone_pkg = get_package_share_directory('sjtu_drone_bringup')
    tb3_pkg = get_package_share_directory('turtlebot3_gazebo')

    # 1. Launch Drone (namespace: /simple_drone)
    drone_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(drone_pkg, 'launch', 'sjtu_drone_gazebo.launch.py')
        ),
    )

    # 2. Launch TurtleBot3 with FORCED namespace using GroupAction
    tb3_bringup = GroupAction([
        PushRosNamespace('simple_robot'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_pkg, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': '3.0',
                'y_pose': '0.0',
            }.items(),
        )
    ])

    # 3. Delayed Takeoff Command (10Hz for 2 seconds)
    takeoff_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', 
            '/simple_drone/takeoff', 
            'std_msgs/msg/Empty', 
            '{}', 
            '-r', '10',
            '-t', '2'
        ],
        shell=True,
        output="screen"
    )

    delayed_takeoff = TimerAction(
        period=5.0,
        actions=[takeoff_cmd]
    )

    nodes_to_run = [
        drone_bringup,
        tb3_bringup,
        delayed_takeoff,
    ]

    return LaunchDescription(nodes_to_run)