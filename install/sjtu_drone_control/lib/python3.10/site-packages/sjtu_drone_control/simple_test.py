import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import time

class SimpleDroneTest(Node):
    def __init__(self):
        super().__init__('simple_drone_test')
        
        # Create publishers for Takeoff and Land
        self.pub_takeoff = self.create_publisher(Empty, '/simple_drone/takeoff', 10)
        self.pub_land = self.create_publisher(Empty, '/simple_drone/land', 10)
        
        self.get_logger().info("Drone Controller Node Started")

    def fly_mission(self):
        # 1. Takeoff
        self.get_logger().info("Taking off...")
        self.pub_takeoff.publish(Empty())
        
        # 2. Hover (Wait 5 seconds)
        self.get_logger().info("Hovering for 5 seconds...")
        time.sleep(5)
        
        # 3. Land
        self.get_logger().info("Landing...")
        self.pub_land.publish(Empty())

def main(args=None):
    rclpy.init(args=args)
    node = SimpleDroneTest()
    
    # Give the connection a moment to establish
    time.sleep(1)
    
    node.fly_mission()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
