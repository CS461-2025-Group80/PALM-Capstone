export GSCAM_CONFIG="nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert"
ros2 run gscam gscam_node   --ros-args   -r __ns:=/camera   -r image_raw:=/camera/image_raw   -p frame_id:=camera_frame


ros2 run elephant_mpeg image_saver_node   --ros-args   -p output_dir:=/home/er/PALM-Capstone/elephant/saved_images