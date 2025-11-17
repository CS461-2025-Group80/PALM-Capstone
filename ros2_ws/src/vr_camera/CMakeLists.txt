cmake_minimum_required(VERSION 3.8)
project(vr_camera)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================
# Find ROS2 Dependencies
# ============================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ============================================
# Find System Dependencies
# ============================================
find_package(OpenXR REQUIRED)

# ============================================
# Library 1: OpenXR Tracker (Pure OpenXR)
# ============================================
add_library(openxr_tracker SHARED
  src/openxr_tracker.cpp
)

target_include_directories(openxr_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(openxr_tracker PUBLIC cxx_std_17)

# Link only with OpenXR (no ROS dependencies)
target_link_libraries(openxr_tracker
  OpenXR::openxr_loader
)

# ============================================
# Library 2: ROS2 Publisher
# ============================================
add_library(ros2_publisher SHARED
  src/ros2_publisher.cpp
)

target_include_directories(ros2_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(ros2_publisher PUBLIC cxx_std_17)

# Link with ROS2 dependencies
ament_target_dependencies(ros2_publisher
  rclcpp
  geometry_msgs
  sensor_msgs
)

# Link with openxr_tracker library
target_link_libraries(ros2_publisher
  openxr_tracker
)

# ============================================
# Main Executable
# ============================================
add_executable(vr_camera
  src/main.cpp
)

target_include_directories(vr_camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(vr_camera PUBLIC cxx_std_17)

# Link with ROS2
ament_target_dependencies(vr_camera
  rclcpp
)

# Link with both custom libraries
target_link_libraries(vr_camera
  openxr_tracker
  ros2_publisher
)

# ============================================
# Install Targets
# ============================================

# Install libraries
install(TARGETS openxr_tracker ros2_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executable
install(TARGETS vr_camera
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# ============================================
# Testing
# ============================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ============================================
# Export Package
# ============================================
ament_package()