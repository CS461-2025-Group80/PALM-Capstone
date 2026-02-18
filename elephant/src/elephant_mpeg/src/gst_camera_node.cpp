#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

class GstCameraNode : public rclcpp::Node
{
public:
    GstCameraNode() : Node("gst_camera_node")
    {
        pub_ = create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

        gst_init(nullptr, nullptr);

        // GStreamer pipeline string
        const char *pipeline_desc =
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1 ! "
            "nvvidconv flip-method=2 ! "
            "video/x-raw,format=BGRx ! "
            "videoconvert ! "
            "video/x-raw,format=RGB ! "
            "appsink name=sink sync=false";

        GError *error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_desc, &error);

        if (!pipeline_ || error) {
            RCLCPP_FATAL(get_logger(), "Failed to create pipeline: %s",
                         error ? error->message : "unknown");
            throw std::runtime_error("Pipeline creation failed");
        }

        // Get appsink
        GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        appsink_ = GST_APP_SINK(sink);

        gst_app_sink_set_emit_signals(appsink_, true);
        gst_app_sink_set_drop(appsink_, true);
        gst_app_sink_set_max_buffers(appsink_, 1);

        g_signal_connect(appsink_, "new-sample",
                         G_CALLBACK(&GstCameraNode::on_new_sample_static),
                         this);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    }

    ~GstCameraNode()
    {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    GstElement *pipeline_;
    GstAppSink *appsink_;

    // Static wrapper for callback
    static GstFlowReturn on_new_sample_static(GstAppSink *sink, gpointer user_data)
    {
        return static_cast<GstCameraNode *>(user_data)->on_new_sample(sink);
    }

    GstFlowReturn on_new_sample(GstAppSink *sink)
    {
        GstSample *sample = gst_app_sink_pull_sample(sink);
        if (!sample)
            return GST_FLOW_ERROR;

        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstCaps *caps = gst_sample_get_caps(sample);

        GstStructure *s = gst_caps_get_structure(caps, 0);
        int width, height;
        gst_structure_get_int(s, "width", &width);
        gst_structure_get_int(s, "height", &height);

        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_READ);

        // Fill ROS2 image message
        auto msg = sensor_msgs::msg::Image();
        msg.header.stamp = now();
        msg.header.frame_id = "camera_frame";

        msg.width = width;
        msg.height = height;
        msg.encoding = "rgb8";
        msg.step = width * 3;
        msg.data.assign(map.data, map.data + map.size);

        pub_->publish(msg);

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        return GST_FLOW_OK;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GstCameraNode>());
    rclcpp::shutdown();
    return 0;
}
