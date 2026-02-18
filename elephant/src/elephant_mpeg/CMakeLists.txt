cmake_minimum_required(VERSION 3.8)
project(gst_camera_pkg)

# Default C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Warnings (optional but recommended)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------
# Find dependencies
# -----------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# GStreamer via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED
    gstreamer-1.0
    gstreamer-app-1.0
)

# Include directories
include_directories(
  include
  ${GSTREAMER_INCLUDE_DIRS}
)

link_directories(${GSTREAMER_LIBRARY_DIRS})

# -----------------------
# Executable
# -----------------------
add_executable(gst_camera_node
  src/gst_camera_node.cpp
)

ament_target_dependencies(
  gst_camera_node
  rclcpp
  sensor_msgs
)

target_link_libraries(
  gst_camera_node
  ${GSTREAMER_LIBRARIES}
)

# -----------------------
# Install
# -----------------------
install(TARGETS
  gst_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------
# Export
# -----------------------
ament_package()
