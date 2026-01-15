# STEERING
conda activate
conda activate aditya
python steering.py
# GStreamer
sudo gst-launch-1.0 -v udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink

