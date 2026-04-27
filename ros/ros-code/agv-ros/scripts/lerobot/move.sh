#!/bin/bash
rm -rf ~/myagv_ros2/src/lerobot_logging
cp -r ../../src/lerobot_logging/ ~/myagv_ros2/src/
cd ~/myagv_ros2
colcon build --packages-select lerobot_logging

