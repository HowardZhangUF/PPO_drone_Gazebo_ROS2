#!/usr/bin/env python3
import sys
import os
import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('spawn_drone')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    content = sys.argv[1]
    namespace = sys.argv[2]

    # ── KEY FIX: if it's a file path, read the actual XML ──
    if os.path.isfile(content):
        node.get_logger().info(f'Reading URDF from file: {content}')
        with open(content, 'r') as f:
            content = f.read()
    
    if not content or not content.strip():
        node.get_logger().error('URDF content is empty! Check the temp file.')
        rclpy.shutdown()
        return

    # Optional position arguments
    x = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    y = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    z = float(sys.argv[5]) if len(sys.argv) > 5 else 0.3

    req = SpawnEntity.Request()
    req.name = namespace
    req.xml = content          # ← now contains actual XML, not a path
    req.robot_namespace = namespace
    req.reference_frame = "world"

    req.initial_pose = Pose()
    req.initial_pose.position.x = x
    req.initial_pose.position.y = y
    req.initial_pose.position.z = z
    req.initial_pose.orientation.w = 1.0

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    node.get_logger().info(f'===== SPAWNING DRONE =====')
    node.get_logger().info(f'Namespace: {namespace}')
    node.get_logger().info(f'Position: ({x:.2f}, {y:.2f}, {z:.2f})')
    node.get_logger().info(f'==========================')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result ' + str(future.result().success) + 
            ' ' + future.result().status_message)
    else:
        node.get_logger().error('Service call failed: %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
