#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

class GStreamerEncoderNode : public rclcpp::Node
{
public:
    GStreamerEncoderNode() : Node("gstreamer_encoder_node")
    {
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/processed", 10,
            std::bind(&GStreamerEncoderNode::callback, this, std::placeholders::_1)
        );

        gst_init(nullptr, nullptr);

        pipeline_ = gst_parse_launch(
            "appsrc name=src is-live=true format=time ! "
            "video/x-raw,format=BGR ! "
            "videoconvert ! "
            "timeoverlay ! "
            "avenc_mpeg2video bitrate=4000000 ! "
            "mpegpsmux ! filesink location=myagv_video.mpg",
            nullptr
        );

        appsrc_ = GST_APP_SRC(
            gst_bin_get_by_name(GST_BIN(pipeline_), "src"));

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    }

    ~GStreamerEncoderNode()
    {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        GstBuffer *buffer =
            gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);

        gst_buffer_fill(buffer, 0, msg->data.data(), msg->data.size());

        GstClockTime ts =
            msg->header.stamp.sec * 1000000000LL +
            msg->header.stamp.nanosec;

        GST_BUFFER_PTS(buffer) = ts;
        GST_BUFFER_DTS(buffer) = ts;

        g_signal_emit_by_name(appsrc_, "push-buffer", buffer, nullptr);
        gst_buffer_unref(buffer);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    GstElement *pipeline_;
    GstAppSrc *appsrc_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GStreamerEncoderNode>());
    rclcpp::shutdown();
    return 0;
}
