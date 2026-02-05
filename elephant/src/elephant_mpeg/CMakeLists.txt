cmake_minimum_required(VERSION 3.8)
project(elephant_mpeg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Standard ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# 2. GStreamer setup (Use PkgConfig instead of find_package)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)
add_executable(camera_gstreamer_node src/camera_gstreamer_node.cpp)
add_executable(opencv_camera_node src/opencv_camera_node.cpp)


# 3. Apply ROS 2 dependencies
ament_target_dependencies(camera_gstreamer_node
    rclcpp 
    sensor_msgs 
)

ament_target_dependencies(opencv_camera_node
    rclcpp
    sensor_msgs
    OpenCV
    cv_bridge
)

# 4. Manually link GStreamer (PkgConfig variables)
target_include_directories(camera_gstreamer_node PUBLIC ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(camera_gstreamer_node ${GSTREAMER_LIBRARIES})
target_link_libraries(opencv_camera_node ${OpenCV_LIBRARIES})

install(TARGETS camera_gstreamer_node opencv_camera_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()