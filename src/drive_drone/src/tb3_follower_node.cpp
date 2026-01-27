#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <cmath>

class TB3FollowerNode : public rclcpp::Node
{
public:
    TB3FollowerNode() : Node("tb3_follower_node")
    {
        // Initialize positions
        tb3_x_ = 0.0;
        tb3_y_ = 0.0;
        drone_x_ = 0.0;
        drone_y_ = 0.0;
        drone_z_ = 0.0;
        drone_ready_ = false;
        takeoff_complete_ = false;
        
        // Following parameters
        follow_height_ = 3.0;  // meters above ground
        takeoff_height_threshold_ = 2.5;  // Consider takeoff complete at this height
        
        // Subscribers
        tb3_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&TB3FollowerNode::tb3OdomCallback, this, std::placeholders::_1));
        
        drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/simple_drone/gt_pose", 10,
            std::bind(&TB3FollowerNode::dronePoseCallback, this, std::placeholders::_1));
        
        // Publishers
        takeoff_pub_ = this->create_publisher<std_msgs::msg::Empty>("/simple_drone/takeoff", 10);
        posctrl_pub_ = this->create_publisher<std_msgs::msg::Bool>("/simple_drone/posctrl", 10);
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/simple_drone/cmd_vel", 10);
        
        // Wait for subscribers to connect
        RCLCPP_INFO(this->get_logger(), "Waiting for drone to be ready...");
        rclcpp::sleep_for(std::chrono::seconds(2));
        
        // Take off
        RCLCPP_INFO(this->get_logger(), "Sending takeoff command...");
        auto takeoff_msg = std_msgs::msg::Empty();
        takeoff_pub_->publish(takeoff_msg);
        
        // Wait for takeoff to complete
        RCLCPP_INFO(this->get_logger(), "Waiting for drone to reach altitude...");
        rclcpp::Rate rate(10);  // 10Hz check rate
        int timeout_counter = 0;
        while (rclcpp::ok() && !takeoff_complete_ && timeout_counter < 50) {  // 5 second timeout
            rclcpp::spin_some(this->get_node_base_interface());
            if (drone_z_ >= takeoff_height_threshold_) {
                takeoff_complete_ = true;
                RCLCPP_INFO(this->get_logger(), 
                    "Takeoff complete! Drone altitude: %.2f m", drone_z_);
            }
            rate.sleep();
            timeout_counter++;
        }
        
        if (!takeoff_complete_) {
            RCLCPP_WARN(this->get_logger(), 
                "Takeoff timeout! Current altitude: %.2f m", drone_z_);
        }
        
        // Enable position control
        RCLCPP_INFO(this->get_logger(), "Enabling position control...");
        auto posctrl_msg = std_msgs::msg::Bool();
        posctrl_msg.data = false;
        posctrl_pub_->publish(posctrl_msg);
        
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        
        // Now start tracking
        drone_ready_ = true;
        
        // Control timer (10Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TB3FollowerNode::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "TB3 Follower started! Drone is now tracking TurtleBot3.");
    }

private:
    void tb3OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tb3_x_ = msg->pose.pose.position.x;
        tb3_y_ = msg->pose.pose.position.y;
    }
    
    void dronePoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        drone_x_ = msg->position.x;
        drone_y_ = msg->position.y;
        drone_z_ = msg->position.z;
    }
    
    void controlLoop()
    {
        // Only track if drone is ready
        if (!drone_ready_) {
            return;
        }
        
        // Calculate distance to TurtleBot
        double dx = tb3_x_ - drone_x_;
        double dy = tb3_y_ - drone_y_;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        // Send position command to follow TB3
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = tb3_x_;
        cmd_msg.linear.y = tb3_y_;
        cmd_msg.linear.z = follow_height_;
        cmd_msg.angular.x = 0.0;
        cmd_msg.angular.y = 0.0;
        cmd_msg.angular.z = 0.0;
        
        cmd_pub_->publish(cmd_msg);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Following TB3 - Distance: %.2f m, Drone: (%.2f, %.2f, %.2f), Target: (%.2f, %.2f, %.2f)",
            distance, drone_x_, drone_y_, drone_z_, tb3_x_, tb3_y_, follow_height_);
    }
    
    // Member variables
    double tb3_x_, tb3_y_;
    double drone_x_, drone_y_, drone_z_;
    double follow_height_;
    double takeoff_height_threshold_;
    bool drone_ready_;
    bool takeoff_complete_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr tb3_odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr drone_pose_sub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr takeoff_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr posctrl_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TB3FollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}