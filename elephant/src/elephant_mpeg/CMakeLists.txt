cmake_minimum_required(VERSION 3.8)
project(elephant_mpeg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------
# Find packages
# ------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# GStreamer via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED
    gstreamer-1.0
    gstreamer-app-1.0
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
)

link_directories(${GSTREAMER_LIBRARY_DIRS})

# ------------------------
# Executable
# ------------------------
add_executable(mpeg_encoder_node
  src/mpeg_encoder_node.cpp
)

ament_target_dependencies(
  mpeg_encoder_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(
  mpeg_encoder_node
  ${OpenCV_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
)

# ------------------------
# Install
# ------------------------
install(TARGETS
  mpeg_encoder_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
