
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <cmath>
#include <memory>

class SimDriveNode : public rclcpp::Node {
public:
    SimDriveNode() : Node("sim_drive_node"), 
                     x_(0.0), y_(0.0), theta_(0.0),
                     linear_vel_x_(0.0), angular_vel_z_(0.0),
                     left_wheel_pos_(0.0), right_wheel_pos_(0.0),
                     last_update_time_(this->get_clock()->now())
    {
        // Declare parameters for wheel geometry
        this->declare_parameter("wheel_radius", 0.033); // meters
        this->declare_parameter("wheel_separation", 0.167); // meters

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SimDriveNode::cmdVelCallback, this, std::placeholders::_1)
        );

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Timer for publishing odometry and joint states
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 Hz update rate
            std::bind(&SimDriveNode::update, this)
        );

        RCLCPP_INFO(this->get_logger(), "SimDriveNode started with differential drive model.");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Store the latest command velocity
        linear_vel_x_ = msg->linear.x;
        angular_vel_z_ = msg->angular.z;
    }

    void update() {
        auto now = this->get_clock()->now();
        double dt = (now - last_update_time_).seconds();
        last_update_time_ = now;

        if (dt == 0.0) return; // Avoid division by zero

        double wheel_radius = this->get_parameter("wheel_radius").as_double();
        double wheel_separation = this->get_parameter("wheel_separation").as_double();

        // --- Inverse Kinematics: Convert robot velocity to wheel velocities ---
        double right_wheel_vel = (linear_vel_x_ + angular_vel_z_ * wheel_separation / 2.0) / wheel_radius;
        double left_wheel_vel = (linear_vel_x_ - angular_vel_z_ * wheel_separation / 2.0) / wheel_radius;

        // Accumulate wheel positions
        left_wheel_pos_ += left_wheel_vel * dt;
        right_wheel_pos_ += right_wheel_vel * dt;

        // --- Forward Kinematics: Update robot pose ---
        double delta_x = linear_vel_x_ * cos(theta_) * dt;
        double delta_y = linear_vel_x_ * sin(theta_) * dt;
        double delta_theta = angular_vel_z_ * dt;

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        // Normalize theta to be within -PI to PI
        theta_ = atan2(sin(theta_), cos(theta_));

        // --- Publish Odometry ---
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "dummy_link"; 

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = linear_vel_x_;
        odom_msg.twist.twist.angular.z = angular_vel_z_;

        odom_pub_->publish(odom_msg);

        // --- Publish TF (odom -> base_link) ---
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = now;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "dummy_link";
        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(odom_tf);

        // --- Publish Joint States ---
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = now;
        joint_state_msg.name = {"LF_wheel_joint", "RF_wheel_joint", "LB_wheel_joint", "RB_wheel_joint"};
        joint_state_msg.position = {left_wheel_pos_, right_wheel_pos_, left_wheel_pos_, right_wheel_pos_};
        joint_state_msg.velocity = {left_wheel_vel, right_wheel_vel, left_wheel_vel, right_wheel_vel};

        joint_state_pub_->publish(joint_state_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_update_time_;
    double x_, y_, theta_; // Robot pose
    double linear_vel_x_, angular_vel_z_; // Current command velocities
    double left_wheel_pos_, right_wheel_pos_; // Accumulated wheel positions
};
    
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimDriveNode>());
    rclcpp::shutdown();
    return 0;
}
