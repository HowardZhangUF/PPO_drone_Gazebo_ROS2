import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('drive_drone')
    
    # Launch configuration variables
    x_pose_1 = LaunchConfiguration('x_pose_1', default='-1.5')
    y_pose_1 = LaunchConfiguration('y_pose_1', default='1.0')
    x_pose_2 = LaunchConfiguration('x_pose_2', default='0.3')
    y_pose_2 = LaunchConfiguration('y_pose_2', default='0.0')

    # TurtleBot3 model path
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    model_folder = 'turtlebot3_burger'
    urdf_path = os.path.join(turtlebot3_gazebo_dir, 'models', model_folder, 'model.sdf')

    # Spawn TB3_1 with unique entity name
    spawn_tb3_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='tb3_1',
        arguments=[
            '-entity', 'tb3_1',  # ✅ Changed from 'burger' to 'tb3_1'
            '-file', urdf_path,
            '-x', x_pose_1,
            '-y', y_pose_1,
            '-z', '0.01',
            '-robot_namespace', 'tb3_1'
        ],
        output='screen',
    )

    # Spawn TB3_2 with unique entity name
    spawn_tb3_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='tb3_2',
        arguments=[
            '-entity', 'tb3_2',  # ✅ Changed from 'burger' to 'tb3_2'
            '-file', urdf_path,
            '-x', x_pose_2,
            '-y', y_pose_2,
            '-z', '0.01',
            '-robot_namespace', 'tb3_2'
        ],
        output='screen',
    )

    # Road network mover for TB3_1
    mover_tb3_1 = Node(
        package='drive_drone',
        executable='tb3_road_network_mover',
        namespace='tb3_1',
        name='road_network_mover',
        parameters=[{
            'robot_namespace': 'tb3_1',
            'start_node': 0
        }],
        output='screen',
    )

    # Road network mover for TB3_2
    mover_tb3_2 = Node(
        package='drive_drone',
        executable='tb3_road_network_mover',
        namespace='tb3_2',
        name='road_network_mover',
        parameters=[{
            'robot_namespace': 'tb3_2',
            'start_node': 5
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('x_pose_1', default_value='-1.5'),
        DeclareLaunchArgument('y_pose_1', default_value='1.0'),
        DeclareLaunchArgument('x_pose_2', default_value='0.3'),
        DeclareLaunchArgument('y_pose_2', default_value='0.0'),
        spawn_tb3_1,
        spawn_tb3_2,
        mover_tb3_1,
        mover_tb3_2,
    ])