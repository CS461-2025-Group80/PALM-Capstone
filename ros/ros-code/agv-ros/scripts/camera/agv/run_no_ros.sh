gst-launch-1.0 nvarguscamerasrc ! \
'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! \
nvvidconv flip-method=2 ! \
xvimagesink

