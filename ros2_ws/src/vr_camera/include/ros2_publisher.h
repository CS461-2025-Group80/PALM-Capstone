// ros2_publisher.h
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "openxr_tracker.h"

class ROS2Publisher : public rclcpp::Node {
public:
    ROS2Publisher();
    ~ROS2Publisher();
    
    void setTracker(OpenXRTracker* tracker) { tracker_ = tracker; }
    void start();

private:
    void timerCallback();
    void publishOrientation(const OrientationData& data);
    void publishPose(const OrientationData& data);
    void publishIMU(const OrientationData& data);
    
    OpenXRTracker* tracker_;
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr orientation_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

