#!/bin/bash

gst-launch-1.0 --gst-plugin-path=install/gst_bridge/lib/gst_bridge/ \
  rosimagesrc ros-topic="/camera/camera/color/image_rect_raw" ! \
  videoconvert ! \
  x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! \
  rtph264pay ! \
  queue ! \
  udpsink host=100.69.210.82 port=5000

