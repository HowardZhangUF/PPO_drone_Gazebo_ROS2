#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <random>
#include <vector>
#include <array>
#include <iostream> 
#include <fstream> 
struct GoalNode {
    int id;
    double x;
    double y;
    double yaw_deg;  // Orientation in degrees
};

class TB3RoadNetworkMover : public rclcpp::Node
{
public:
    TB3RoadNetworkMover() : Node("tb3_road_network_mover")
    {
        std::cout << "[DEBUG] Constructor started" << std::endl;
        try {  
            // Declare namespace parameter
            this->declare_parameter<std::string>("robot_namespace", "");
            robot_namespace_ = this->get_parameter("robot_namespace").as_string();
            
            // Initialize random number generator
            rng_.seed(std::random_device{}());
            
            // Initialize state
            current_x_ = 0.0;
            current_y_ = 0.0;
            current_yaw_ = 0.0;
            goal_x_ = 0.0;
            goal_y_ = 0.0;
            position_received_ = false;
            std::cout << "[DEBUG] Initializing state variables" << std::endl; 
            // Control parameters
            max_linear_vel_ = 0.22;   // m/s (TurtleBot3 max speed)
            max_angular_vel_ = 2.84;  // rad/s
            goal_tolerance_ = 0.15;   // meters
            
            // PID gains
            kp_linear_ = 1.0;
            kp_angular_ = 4.0;
             std::cout << "[DEBUG] Creating road network" << std::endl; 
            
            // ═══════════════════════════════════════════════════════
            // ROAD NETWORK DEFINITION (road_network_V2)
            // ═══════════════════════════════════════════════════════
            road_network_.push_back({0, -1.5,  1.0, 180.0});
            road_network_.push_back({1,  0.0,  1.0, 180.0});
            road_network_.push_back({2,  2.1,  1.0,  90.0});
            road_network_.push_back({3, -1.5, -1.0, 270.0});
            road_network_.push_back({4,  0.0, -1.0,  90.0});
            road_network_.push_back({5,  0.3,  0.0,   0.0});
            road_network_.push_back({6,  1.7,  0.0,   0.0});
            road_network_.push_back({7,  2.1, -1.0, 270.0});
            
            n_states_ = road_network_.size();
            std::cout << "[DEBUG] Road network size: " << n_states_ << std::endl; 
            std::cout << "[DEBUG] Initializing transition matrix" << std::endl;
            // Initialize transition matrix with proper size FIRST
            transition_matrix_.resize(n_states_, std::vector<double>(n_states_, 0.0));
            
            // Now populate the transition matrix (8x8)
            // Probability of going from state i to state j
            transition_matrix_.resize(n_states_);
            for (size_t i = 0; i < n_states_; ++i) {
                transition_matrix_[i].resize(n_states_, 0.0);
            }
            
            std::cout << "[DEBUG] Populating transition matrix" << std::endl;  // ← ADD THIS LINE
            
            // Row 0
            transition_matrix_[0][4] = 1.0;
            // Row 1
            transition_matrix_[1][0] = 0.5;
            transition_matrix_[1][3] = 0.5;
            // Row 2
            transition_matrix_[2][1] = 1.0;
            // Row 3
            transition_matrix_[3][0] = 1.0;
            // Row 4
            transition_matrix_[4][5] = 1.0;
            // Row 5
            transition_matrix_[5][6] = 1.0;
            // Row 6
            transition_matrix_[6][2] = 1.0;
            // Row 7
            transition_matrix_[7][4] = 1.0;
        
            std::cout << "[DEBUG] Transition matrix populated" << std::endl;
            // Start at node 5 (middle of road network)
            prev_goal_idx_ = 5;
            current_goal_idx_ = 5;

            
            std::cout << "[DEBUG] Creating publishers" << std::endl;
            // ═══════════════════════════════════════════════════════
            // ROS PUBLISHERS & SUBSCRIBERS
            // ═══════════════════════════════════════════════════════
            
            // Publisher for TurtleBot velocity commands (relative topic for namespacing)
            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "cmd_vel", 10);
            
            // Publisher for current goal (for visualization, relative topic)
            goal_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
                "current_goal", 10);
            std::cout << "[DEBUG] Creating subscriber" << std::endl; 

            std::cout << "[DEBUG] Setting initial goal" << std::endl; 
            updateGoal(current_goal_idx_);

            std::cout << "[DEBUG] Creating subscriber" << std::endl;
            // Subscriber for odometry (relative topic for namespacing)
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "odom", 10,
                std::bind(&TB3RoadNetworkMover::odomCallback, this, std::placeholders::_1));
            
            std::cout << "[DEBUG] Creating control timer" << std::endl; 
            // Control timer (20Hz for smooth motion)
            control_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(&TB3RoadNetworkMover::controlLoop, this));
            
            std::cout << "[DEBUG] Creating goal timer" << std::endl;    
            // Goal update timer (10Hz for Markov chain updates)
            goal_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&TB3RoadNetworkMover::markovGoalUpdate, this));
            
            std::cout << "[DEBUG] Constructor complete" << std::endl;
            RCLCPP_INFO(this->get_logger(), 
                "═══════════════════════════════════════════════════");
            RCLCPP_INFO(this->get_logger(), 
                "TB3 Road Network Mover with Markov Chain");
            if (!robot_namespace_.empty()) {
                RCLCPP_INFO(this->get_logger(), "Namespace: %s", robot_namespace_.c_str());
            }
            RCLCPP_INFO(this->get_logger(), 
                "═══════════════════════════════════════════════════");
            RCLCPP_INFO(this->get_logger(), "Road network: %zu nodes", n_states_);
            RCLCPP_INFO(this->get_logger(), "Starting at node %d: (%.2f, %.2f)", 
                current_goal_idx_, goal_x_, goal_y_);
        }catch (const std::exception& e) {  // ← ADD THIS CLOSING BRACE AND CATCH
        std::cerr << "[ERROR] Exception in constructor: " << e.what() << std::endl;
        throw;
    }
    }

private:
    // ═══════════════════════════════════════════════════════
    // MARKOV CHAIN GOAL SELECTION
    // ═══════════════════════════════════════════════════════
    
    void markovGoalUpdate()
    {
        if (!position_received_) {
            return;
        }
        
        // Calculate distance to current goal
        double dx = goal_x_ - current_x_;
        double dy = goal_y_ - current_y_;
        double dist_to_goal = std::sqrt(dx*dx + dy*dy);
        
        // Check if goal reached (within tolerance)
        if (dist_to_goal < goal_tolerance_) {
            // Sample next goal from transition matrix
            int next_goal = sampleNextState(current_goal_idx_);
            
            if (next_goal != current_goal_idx_) {
                prev_goal_idx_ = current_goal_idx_;
                current_goal_idx_ = next_goal;
                updateGoal(next_goal);
                
                RCLCPP_INFO(this->get_logger(), 
                    "🎯 New goal: Node %d → (%.2f, %.2f)", 
                    next_goal, goal_x_, goal_y_);
            }
        }
    }
    
    int sampleNextState(int current_state)
    {
        // Bounds checking
        if (current_state < 0 || current_state >= static_cast<int>(n_states_)) {
            RCLCPP_ERROR(this->get_logger(), 
                "Invalid state index: %d (max: %zu)", current_state, n_states_);
            return current_state;
        }
        
        // Get transition probabilities for current state
        const auto& probs = transition_matrix_[current_state];
        
        // Verify probabilities sum to ~1.0
        double sum = 0.0;
        for (double p : probs) {
            sum += p;
        }
        
        if (std::abs(sum - 1.0) > 0.01) {
            RCLCPP_WARN(this->get_logger(), 
                "Transition probabilities for state %d sum to %.3f (expected 1.0)", 
                current_state, sum);
        }
        
        // Create discrete distribution
        std::discrete_distribution<int> dist(probs.begin(), probs.end());
        
        // Sample next state
        return dist(rng_);
    }
    
    void updateGoal(int goal_idx)
    {
        if (goal_idx >= 0 && goal_idx < static_cast<int>(road_network_.size())) {
            const auto& node = road_network_[goal_idx];
            goal_x_ = node.x;
            goal_y_ = node.y;
            
            // Publish goal for visualization
            auto goal_msg = geometry_msgs::msg::Pose();
            goal_msg.position.x = goal_x_;
            goal_msg.position.y = goal_y_;
            goal_msg.position.z = 0.0;
            
            // Convert yaw to quaternion
            double yaw_rad = node.yaw_deg * M_PI / 180.0;
            goal_msg.orientation.x = 0.0;
            goal_msg.orientation.y = 0.0;
            goal_msg.orientation.z = std::sin(yaw_rad / 2.0);
            goal_msg.orientation.w = std::cos(yaw_rad / 2.0);
            
            goal_pub_->publish(goal_msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                "Invalid goal index: %d (max: %zu)", goal_idx, road_network_.size());
        }
    }
    
    // ═══════════════════════════════════════════════════════
    // ODOMETRY CALLBACK
    // ═══════════════════════════════════════════════════════
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        // Extract yaw from quaternion
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        current_yaw_ = std::atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz));
        
        if (!position_received_) {
            position_received_ = true;
            RCLCPP_INFO(this->get_logger(), 
                "✓ First position received: [%.2f, %.2f]m", 
                current_x_, current_y_);
        }
    }
    
    // ═══════════════════════════════════════════════════════
    // MOTION CONTROL
    // ═══════════════════════════════════════════════════════
    
    void controlLoop()
    {
        if (!position_received_) {
            stopRobot();
            return;
        }
        
        // ═══════════════════════════════════════════════════════
        // STEP 1: Calculate vector to goal
        // ═══════════════════════════════════════════════════════
        double dx = goal_x_ - current_x_;          // X component of vector to goal
        double dy = goal_y_ - current_y_;          // Y component of vector to goal
        double distance = std::sqrt(dx*dx + dy*dy); // Euclidean distance
        
        auto cmd_msg = geometry_msgs::msg::Twist();
        
        if (distance < goal_tolerance_) {
            // ═══════════════════════════════════════════════════
            // AT GOAL: Stop completely
            // ═══════════════════════════════════════════════════
            stopRobot();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "✓ At node %d (%.3f m from goal)", current_goal_idx_, distance);
            return;
        }
        
        // ═══════════════════════════════════════════════════════
        // STEP 2: Calculate desired world velocities
        // ═══════════════════════════════════════════════════════
        double speed = std::min(kp_linear_ * distance, max_linear_vel_);
        double vx_world = (dx / distance) * speed;  // Normalized direction * speed
        double vy_world = (dy / distance) * speed;
        
        // ═══════════════════════════════════════════════════════
        // STEP 3: Transform to robot frame (CRITICAL!)
        // ═══════════════════════════════════════════════════════
        // Robot's local frame:
        //   X-axis = forward direction (where robot faces)
        //   Y-axis = left direction (perpendicular to forward)
        //
        // World velocities → Robot velocities:
        //   v_forward = vx_world*cos(yaw) + vy_world*sin(yaw)
        //   v_left    = -vx_world*sin(yaw) + vy_world*cos(yaw)
        
        double cos_yaw = std::cos(current_yaw_);
        double sin_yaw = std::sin(current_yaw_);
        
        double v_forward = vx_world * cos_yaw + vy_world * sin_yaw;  // Forward velocity
        double v_left = -vx_world * sin_yaw + vy_world * cos_yaw;    // Sideways velocity
        
        // ═══════════════════════════════════════════════════════
        // STEP 4: Convert to linear + angular velocities
        // ═══════════════════════════════════════════════════════
        // TurtleBot3 is differential drive (2 wheels):
        //   linear.x  = forward speed
        //   angular.z = rotation speed (to handle sideways motion)
        
        cmd_msg.linear.x = v_forward;  // Use forward component directly
        
        // If trying to move sideways, rotate to face that direction
        if (std::abs(v_left) > 0.01) {
            // Proportional rotation to cancel sideways motion
            cmd_msg.angular.z = 2.0 * v_left;  // Lower gain = smoother turning
            cmd_msg.angular.z = std::clamp(cmd_msg.angular.z, -max_angular_vel_, max_angular_vel_);
        } else {
            cmd_msg.angular.z = 0.0;
        }
        
        // ═══════════════════════════════════════════════════════
        // STEP 5: Log and publish
        // ═══════════════════════════════════════════════════════
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "→ Node %d | Dist: %.2f m | Vel: [fwd=%.2f, ang=%.2f] | World: [%.2f, %.2f]",
            current_goal_idx_, distance, 
            cmd_msg.linear.x, cmd_msg.angular.z,
            vx_world, vy_world);
        
        cmd_vel_pub_->publish(cmd_msg);
    }
    
    void stopRobot()
    {
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_msg);
    }
    
    // ═══════════════════════════════════════════════════════
    // MEMBER VARIABLES
    // ═══════════════════════════════════════════════════════
    
    // Current state
    double current_x_, current_y_, current_yaw_;
    double goal_x_, goal_y_;
    bool position_received_;
    
    // Markov chain state
    int prev_goal_idx_;
    int current_goal_idx_;
    size_t n_states_;
    
    // Road network
    std::vector<GoalNode> road_network_;
    std::vector<std::vector<double>> transition_matrix_;
    
    // Random number generator
    std::mt19937 rng_;
    
    // Control parameters
    double max_linear_vel_, max_angular_vel_;
    double goal_tolerance_;
    double kp_linear_, kp_angular_;
    
    // Namespace
    std::string robot_namespace_;
    
    // ROS components
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goal_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr goal_timer_;
    std::vector<double> trajectory_x_;  // X positions over time
    std::vector<double> trajectory_y_;  // Y positions over time
    std::vector<double> trajectory_t_;  // Timestamps (seconds)

};

int main(int argc, char** argv)
{
    std::cout << "[DEBUG] main() started" << std::endl;  // ← ADD THIS LINE
    
    std::cout << "[DEBUG] Calling rclcpp::init()" << std::endl;
    rclcpp::init(argc, argv);
    
    try {
        std::cout << "[DEBUG] Creating node" << std::endl;  // ← ADD THIS LINE
        auto node = std::make_shared<TB3RoadNetworkMover>();
        
        std::cout << "[DEBUG] Starting spin" << std::endl;  // ← ADD THIS LINE
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception in main: " << e.what() << std::endl;  // ← ADD THIS LINE
        RCLCPP_ERROR(rclcpp::get_logger("tb3_road_network_mover"), 
            "Exception caught: %s", e.what());
        rclcpp::shutdown();  // ← ADD THIS LINE
        return 1;
    }
    
    std::cout << "[DEBUG] Shutting down" << std::endl;  // ← ADD THIS LINE
    rclcpp::shutdown();
    return 0;
}