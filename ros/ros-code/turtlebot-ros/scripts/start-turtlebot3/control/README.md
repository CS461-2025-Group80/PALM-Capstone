
# udp_to_cmd_vel.py
This file, when run with Python, will listen for UDP packets being sent to a specific port (see the file).
The "cmd_vel" topic on ROS will be modified based on these packets, thus making the robot move.

python3 udp_to_cmd_vel.py
