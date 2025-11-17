// ros2_publisher.cpp
#include "ros2_publisher.h"

ROS2Publisher::ROS2Publisher() 
    : Node("openxr_ros2_bridge"), tracker_(nullptr) {
    
    // Create publishers
    orientation_pub_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
        "quest/orientation", 10);
    
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "quest/pose", 10);
    
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "quest/imu", 10);
    
    RCLCPP_INFO(this->get_logger(), "ROS2 Publisher initialized");
}

ROS2Publisher::~ROS2Publisher() {}

void ROS2Publisher::start() {
    // Create timer for publishing at 100Hz
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ROS2Publisher::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Started publishing at 100Hz");
}

void ROS2Publisher::timerCallback() {
    if (!tracker_ || !tracker_->isRunning()) {
        return;
    }
    
    OrientationData data;
    if (tracker_->update(data)) {
        publishOrientation(data);
        publishPose(data);
        publishIMU(data);
    }
}

void ROS2Publisher::publishOrientation(const OrientationData& data) {
    if (!data.orientation_valid) return;
    
    auto msg = geometry_msgs::msg::QuaternionStamped();
    msg.header.stamp = this->now();
    msg.header.frame_id = "quest_head";
    msg.quaternion.x = data.x;
    msg.quaternion.y = data.y;
    msg.quaternion.z = data.z;
    msg.quaternion.w = data.w;
    
    orientation_pub_->publish(msg);
}

void ROS2Publisher::publishPose(const OrientationData& data) {
    if (!data.orientation_valid && !data.position_valid) return;
    
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.stamp = this->now();
    msg.header.frame_id = "quest_base";
    
    if (data.orientation_valid) {
        msg.pose.orientation.x = data.x;
        msg.pose.orientation.y = data.y;
        msg.pose.orientation.z = data.z;
        msg.pose.orientation.w = data.w;
    }
    
    if (data.position_valid) {
        msg.pose.position.x = data.pos_x;
        msg.pose.position.y = data.pos_y;
        msg.pose.position.z = data.pos_z;
    }
    
    pose_pub_->publish(msg);
}

void ROS2Publisher::publishIMU(const OrientationData& data) {
    if (!data.orientation_valid) return;
    
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = "quest_head";
    
    msg.orientation.x = data.x;
    msg.orientation.y = data.y;
    msg.orientation.z = data.z;
    msg.orientation.w = data.w;
    
    // Set covariance
    msg.orientation_covariance[0] = 0.01;
    msg.orientation_covariance[4] = 0.01;
    msg.orientation_covariance[8] = 0.01;
    
    // No angular velocity or linear acceleration data
    msg.angular_velocity_covariance[0] = -1;
    msg.linear_acceleration_covariance[0] = -1;
    
    imu_pub_->publish(msg);
}
