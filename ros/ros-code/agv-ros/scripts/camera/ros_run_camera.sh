ros2 run gscam gscam_node --ros-args \
  -p gscam_config:="nvarguscamerasrc ! video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1 ! nvvidconv flip-method=2 ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR" \
  -p camera_name:=camera \
  -p frame_id:=camera