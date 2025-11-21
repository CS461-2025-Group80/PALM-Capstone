#!/bin/bash

sudo apt update

# generic ROS packages
sudo apt-get install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro ros-humble-ros-gz* ros-humble-*-ros-control ros-humble-joint-state-publisher-gui ros-humble-turtlesim ros-humble-robot-localization ros-humble-joy ros-humble-joy-teleop ros-humble-tf-transformations ros-humble-urdf-tutorial python3-pip

pip install transforms3d

# for gstreamer
sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

