cmake_minimum_required(VERSION 3.8)
project(elephant_mpeg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 2. Define the Executable
# Assumes your C++ file is named src/mpeg_recorder.cpp
add_executable(mpeg_recorder src/mpeg_recorder.cpp)

# 3. Include Directories and Link Libraries
target_include_directories(mpeg_recorder PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(mpeg_recorder
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 4. Install instructions (so 'ros2 run' can find it)
install(TARGETS
  mpeg_recorder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()