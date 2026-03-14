#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# -*- coding: utf-8 -*-
import sys
import os
import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('spawn_drone')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    # First arg can be content OR file path
    content_or_path = sys.argv[1]
    namespace = sys.argv[2]
    
    # Optional position arguments (x, y, z)
    x = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    y = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    z = float(sys.argv[5]) if len(sys.argv) > 5 else 0.3

    # Check if it's a file path or content
    if os.path.isfile(content_or_path):
        node.get_logger().info(f'Reading URDF from file: {content_or_path}')
        with open(content_or_path, 'r') as f:
            content = f.read()
    else:
        node.get_logger().info('Using URDF content directly')
        content = content_or_path

    req = SpawnEntity.Request()
    req.name = namespace
    req.xml = content
    req.robot_namespace = namespace
    req.reference_frame = "world"
    
    # Set initial pose
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
            'Result ' + str(future.result().success) + " " + future.result().status_message)
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
