# ROS2
## ROS2 Installation
ROS2 is just a bunch of Linux packages.

Follow these [instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). We use ROS2 Humble Hawksbill for Ubuntu 22.04. You can also just run the install script called "./palm-project/scripts/install-ros2-humble.bash" on any Ubuntu system.
## ROS2 Packages
There are quite a lot of packages we'll be using. Just run "./palm-project/scripts/install-ros2-packages.bash".
### rosdep
#### sudo rosdep init
Initializes your ROS dependency checker.
#### sudo rosdep update
Updates your ROS dependency checker.
#### sudo rosdep install --from-paths ./src --ignore-src -r -y
Installs all dependencies from the "./src" directory.
## Sourcing
### . install/setup.bash
Adds the packages in src to the system's ROS2 environment (for the current terminal session)
### ros2 pkg list
Lists all the ROS2 packages that your terminal sees.
### ros2 run package-name script-in-package-setup-name
Runs a specific package's script. This typically runs a Node.
## Building
### colcon build
This will use "colcon", a build tool, to build your ROS2 packages (under src).
## Topics
### ros2 topic list
Will list all topics ROS2 current knows about.
### ros2 topic echo /topic-name
Will echo whatever is being written to the topic.
### ros2 topic info /topic-name --verbose
Will give a lot of data about a specific topic (including the number of publishers and subscribers).
### ros2 topic pub /topic-name data-type "data: ..."
Will continually publish a hard-set value to a specific topic.

Example: ros2 topic pub /topic-name std\_msgs/msg/String "data: 'hello'"

