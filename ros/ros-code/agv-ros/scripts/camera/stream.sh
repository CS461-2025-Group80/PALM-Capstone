ros2 run camera_stream stream_node \
  --ros-args \
  -p image_topic:=/camera/image_raw \
  -p jpeg_quality:=10 \
  -p http_port:=8080