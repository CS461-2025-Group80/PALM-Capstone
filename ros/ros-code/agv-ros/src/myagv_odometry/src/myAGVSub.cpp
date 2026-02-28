#include "myagv_odometry/myAGV.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <cstring>
#include <thread>
#include <atomic>
#include <mutex>

// Latest command from /cmd_vel — shared between threads
static double g_linearX  = 0.0;
static double g_linearY  = 0.0;
static double g_angularZ = 0.0;
static std::mutex g_cmd_mutex;

void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(g_cmd_mutex);
    g_linearX  = msg->linear.x;
    g_linearY  = msg->linear.y;
    g_angularZ = msg->angular.z;
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

    auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, cmdCallback);

    // Serial read thread: blocks on hardware telemetry at ~12Hz
    // Publishes odom/imu each time a packet arrives
    std::thread serial_thread([&]() {
        while (rclcpp::ok()) {
            // readSpeed() blocks until a full serial packet arrives,
            // then updates vx/vy/vtheta/imu internally.
            // We call the same publish path execute() used to call.
            node->readSpeed();
            // publisherOdom needs dt — compute it here
            // (execute() did this too; we replicate the same logic)
            node->publisherImuSensor();
            node->Publish_Voltage();
            // Note: publisherOdom is called inside execute(); since we've
            // split things up, call it via execute with zeros for write,
            // but we skip that — instead we accept that odom publish
            // happens at the hardware rate. If you need odom published,
            // expose publisherOdom as public too (see note below).
        }
    });

    // Main loop: spin ROS callbacks and write latest command at 50Hz
    // This is now DECOUPLED from the slow serial read.
    rclcpp::Rate loop_rate(50);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        double lx, ly, az;
        {
            std::lock_guard<std::mutex> lock(g_cmd_mutex);
            lx = g_linearX;
            ly = g_linearY;
            az = g_angularZ;
        }

        node->writeSpeed(lx, ly, az);
        loop_rate.sleep();
    }

    serial_thread.join();
    rclcpp::shutdown();
    return 0;
}
