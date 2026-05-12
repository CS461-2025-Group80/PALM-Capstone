#!/bin/bash
rm -rf ~/myagv_ros2/src/camera_stream
cp -r ../../src/camera_stream/ ~/myagv_ros2/src/
cd ~/myagv_ros2
colcon build --packages-select camera_stream

