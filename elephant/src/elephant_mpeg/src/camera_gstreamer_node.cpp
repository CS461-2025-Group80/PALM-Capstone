#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

class GStreamerCameraNode : public rclcpp::Node {
public:
    GStreamerCameraNode() : Node("gstreamer_camera_node") {
        // Subscribe to ROS2 camera topic
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&GStreamerCameraNode::imageCallback, this, std::placeholders::_1)
        );

        // Initialize GStreamer pipeline
        gst_init(nullptr, nullptr);

        std::string pipeline_str =
            "appsrc name=mysrc ! videoconvert ! "
            "timeoverlay ! "
            "avenc_mpeg2video bitrate=4000000 ! "
            "mpegpsmux ! filesink location=myagv_video.mpg";

        GError* error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        if (!pipeline_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create GStreamer pipeline");
            exit(-1);
        }

        appsrc_ = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline_), "mysrc"));
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    }

    ~GStreamerCameraNode() {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Wrap OpenCV Mat into GstBuffer
        int size = frame.total() * frame.elemSize();
        GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
        gst_buffer_fill(buffer, 0, frame.data, size);

        // Set PTS/DTS from ROS timestamp
        GST_BUFFER_PTS(buffer) = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
        GST_BUFFER_DTS(buffer) = GST_BUFFER_PTS(buffer);

        GstFlowReturn ret;
        g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
        gst_buffer_unref(buffer);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    GstElement* pipeline_;
    GstAppSrc* appsrc_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GStreamerCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
