ros2 run gscam gscam_node \
  --ros-args \
  -p gscam_config:="v4l2src device=/dev/video0 ! video/x-raw, width=1920, height=1080, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! appsink" \
  -p frame_id:="camera_frame" \
  -p use_gst_timestamps:=false