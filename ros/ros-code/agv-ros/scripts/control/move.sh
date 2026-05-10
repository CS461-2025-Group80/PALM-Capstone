#!/bin/bash
rm -rf ~/myagv_ros2/src/myagv_odometry
cp -r ../../src/myagv_odometry ~/myagv_ros2/src/
cd ~/myagv_ros2
colcon build --packages-select myagv_odometry

