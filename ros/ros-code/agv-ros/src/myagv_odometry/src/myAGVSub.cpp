#include "myagv_odometry/myAGV.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <cstring>

double linearX = 0.0;
double linearY = 0.0;
double angularZ = 0.0;

void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    linearX = msg->linear.x;
    linearY = msg->linear.y;
    angularZ = msg->angular.z;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyAGV>("myagv_odometry_node");

    if (!node->init()) {
        RCLCPP_ERROR(node->get_logger(), "myAGV initialized failed!");
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "myAGV initialized successful!");

    // Queue depth 1: DDS discards stale messages automatically
    auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, cmdCallback);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    rclcpp::Rate loop_rate(100);

    while (rclcpp::ok()) {
        // Drain all pending callbacks before execute()
        executor.spin_some(std::chrono::milliseconds(0));
        node->execute(linearX, linearY, angularZ);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
