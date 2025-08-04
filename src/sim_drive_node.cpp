#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SimDriveNode : public rclcpp::Node {
public:
    SimDriveNode() : Node("sim_drive_node") {
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SimDriveNode::cmdVelCallback, this, std::placeholders::_1)
        );

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        RCLCPP_INFO(this->get_logger(), "SimDriveNode started (CANless)");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Gazebo에서 /cmd_vel로 제어받고 /odom은 Gazebo plugin에서 따로 퍼블리시됨
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear=%.2f angular=%.2f",
                    msg->linear.x, msg->angular.z);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};
    
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimDriveNode>());
    rclcpp::shutdown();
    return 0;
}
