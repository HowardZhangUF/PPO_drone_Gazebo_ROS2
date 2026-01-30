import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/user/PPO_drone_Gazebo_ROS2/install/sjtu_drone_bringup'
