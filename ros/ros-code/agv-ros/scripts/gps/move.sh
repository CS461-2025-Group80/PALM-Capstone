#!/bin/bash
rm -rf ~/myagv_ros2/src/um982_gps
cp -r ../../src/um982_gps ~/myagv_ros2/src/
cd ~/myagv_ros2
colcon build --packages-select um982_gps
