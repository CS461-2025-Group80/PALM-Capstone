#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <cv_bridge/cv_bridge.h>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

class MPEGEncoderNode : public rclcpp::Node
{
public:
    MPEGEncoderNode()
        : Node("mpeg_encoder_node")
    {
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image",
            10,
            std::bind(&MPEGEncoderNode::image_callback,this,std::placeholders::_1));

        pub_ = create_publisher<
            sensor_msgs::msg::CompressedImage>("/camera/mpeg",10);

        gst_init(nullptr,nullptr);

        std::string pipeline_desc =
            "appsrc name=src is-live=true block=true format=time "
            "! videoconvert "
            "! x264enc tune=zerolatency bitrate=2048 speed-preset=ultrafast "
            "! video/x-h264,stream-format=byte-stream "
            "! appsink name=sink";

        pipeline_ = gst_parse_launch(pipeline_desc.c_str(), nullptr);

        appsrc_ = GST_APP_SRC(
            gst_bin_get_by_name(GST_BIN(pipeline_), "src"));

        appsink_ = GST_APP_SINK(
            gst_bin_get_by_name(GST_BIN(pipeline_), "sink"));

        gst_app_sink_set_emit_signals(appsink_, false);
        gst_app_sink_set_drop(appsink_, true);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    }

    ~MPEGEncoderNode()
    {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image → cv::Mat
        cv::Mat frame =
            cv_bridge::toCvCopy(msg, "bgr8")->image;

        int size = frame.total() * frame.elemSize();

        GstBuffer *buffer =
            gst_buffer_new_allocate(NULL, size, NULL);

        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_WRITE);
        memcpy(map.data, frame.data, size);
        gst_buffer_unmap(buffer, &map);

        GST_BUFFER_PTS(buffer) =
            gst_util_uint64_scale(
                msg->header.stamp.nanosec,
                GST_SECOND,
                1000000000);

        gst_app_src_push_buffer(appsrc_, buffer);

        // Pull encoded output
        GstSample *sample =
            gst_app_sink_try_pull_sample(appsink_,0);

        if(sample)
        {
            GstBuffer *enc_buf =
                gst_sample_get_buffer(sample);

            GstMapInfo enc_map;

            if(gst_buffer_map(enc_buf,&enc_map,GST_MAP_READ))
            {
                sensor_msgs::msg::CompressedImage out;

                out.header = msg->header;
                out.format = "h264";

                out.data.assign(
                    enc_map.data,
                    enc_map.data + enc_map.size);

                pub_->publish(out);

                gst_buffer_unmap(enc_buf,&enc_map);
            }

            gst_sample_unref(sample);
        }
    }

    rclcpp::Subscription<
        sensor_msgs::msg::Image>::SharedPtr sub_;

    rclcpp::Publisher<
        sensor_msgs::msg::CompressedImage>::SharedPtr pub_;

    GstElement *pipeline_;
    GstAppSrc *appsrc_;
    GstAppSink *appsink_;
};

int main(int argc,char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPEGEncoderNode>());
    rclcpp::shutdown();
    return 0;
}
