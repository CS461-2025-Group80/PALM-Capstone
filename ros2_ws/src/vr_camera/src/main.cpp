// main.cpp
#include "openxr_tracker.h"
#include "ros2_publisher.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Create OpenXR tracker
    auto tracker = std::make_unique<OpenXRTracker>();
    
    if (!tracker->initialize()) {
        std::cerr << "Failed to initialize OpenXR: " 
                  << tracker->getLastError() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    
    std::cout << "OpenXR initialized successfully" << std::endl;
    
    // Create ROS2 publisher node
    auto publisher = std::make_shared<ROS2Publisher>();
    publisher->setTracker(tracker.get());
    publisher->start();
    
    // Spin
    rclcpp::spin(publisher);
    
    // Cleanup
    tracker->cleanup();
    rclcpp::shutdown();
    
    return 0;
}