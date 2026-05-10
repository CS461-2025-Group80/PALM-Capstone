# move_to_working_directory.sh
Deletes currently live lerobot logging code (in ~/myagv_ros2/src/lerobot_logging)
Moves code in the repository to the live ROS directory (from ../../src/lerobot_logging to ~/myagv_ros2/src/lerobot_logging)

# run_lerobotlogger.sh
## argument 1 ($1)
A string. The name of the session.
## argument 2 ($2)
An integer. 0 if we are NOT using ROS for the camera and 1 if we are using ROS for the camera.
## argument 3 ($3)
A float. 0 if we're just using 1 session. Otherwise, this represents the number of minutes a session will last. If you exceed this limit, a new session directory will be created under this program's current run.