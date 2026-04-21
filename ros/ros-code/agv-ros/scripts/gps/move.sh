#!/bin/bash
sudo chmod 666 /dev/ttyUSB0
rm -rf ~/myagv_ros2/src/um982_gps_manual
cp -r ../../src/um982_gps_manual ~/myagv_ros2/src/
cd ~/myagv_ros2
colcon build --packages-select um982_gps_manual
