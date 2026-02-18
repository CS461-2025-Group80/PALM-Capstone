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
        : Node("mpeg_encoder_node"), pipeline_ready_(false)
    {
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image",
            rclcpp::SensorDataQoS(),
            std::bind(&MPEGEncoderNode::image_callback,
                      this,
                      std::placeholders::_1));

        pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "/camera/mpeg", 10);

        gst_init(nullptr, nullptr);
    }

    ~MPEGEncoderNode()
    {
        if(pipeline_)
        {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
    }

private:

    // ---------- Build pipeline once we know frame size ----------
    void init_pipeline(int width, int height)
    {
        std::stringstream ss;

        ss <<
        "appsrc name=src is-live=true block=true format=time "
        "! videoconvert "
        "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2048 key-int-max=30 "
        "! video/x-h264,stream-format=byte-stream,alignment=au "
        "! appsink name=sink sync=false";

        pipeline_ = gst_parse_launch(ss.str().c_str(), nullptr);

        appsrc_ = GST_APP_SRC(
            gst_bin_get_by_name(GST_BIN(pipeline_), "src"));

        appsink_ = GST_APP_SINK(
            gst_bin_get_by_name(GST_BIN(pipeline_), "sink"));

        // ---- VERY IMPORTANT: caps describing incoming frames ----
        GstCaps *caps = gst_caps_new_simple(
            "video/x-raw",
            "format", G_TYPE_STRING, "BGR",
            "width", G_TYPE_INT, width,
            "height", G_TYPE_INT, height,
            "framerate", GST_TYPE_FRACTION, 30, 1,
            NULL);

        gst_app_src_set_caps(appsrc_, caps);
        gst_caps_unref(caps);

        gst_app_sink_set_emit_signals(appsink_, false);
        gst_app_sink_set_drop(appsink_, true);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        pipeline_ready_ = true;

        RCLCPP_INFO(get_logger(),"GStreamer pipeline initialized");
    }

    // ---------- Image callback ----------
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        if(!pipeline_ready_)
            init_pipeline(frame.cols, frame.rows);

        int size = frame.total() * frame.elemSize();

        GstBuffer *buffer = gst_buffer_new_allocate(NULL, size, NULL);

        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_WRITE);
        memcpy(map.data, frame.data, size);
        gst_buffer_unmap(buffer, &map);

        // Timestamp handling
        uint64_t timestamp =
            msg->header.stamp.sec * GST_SECOND +
            msg->header.stamp.nanosec;

        GST_BUFFER_PTS(buffer) = timestamp;
        GST_BUFFER_DURATION(buffer) =
            gst_util_uint64_scale_int(1, GST_SECOND, 30);

        gst_app_src_push_buffer(appsrc_, buffer);

        // ---------- Pull ALL encoded output ----------
        while(true)
        {
            GstSample *sample =
                gst_app_sink_try_pull_sample(appsink_, 0);

            if(!sample)
                break;

            GstBuffer *enc_buf =
                gst_sample_get_buffer(sample);

            GstMapInfo enc_map;

            if(gst_buffer_map(enc_buf, &enc_map, GST_MAP_READ))
            {
                sensor_msgs::msg::CompressedImage out;
                out.header = msg->header;
                out.format = "h264";

                out.data.assign(
                    enc_map.data,
                    enc_map.data + enc_map.size);

                    pub_->publish(out);

                    // ---- Print publish time ----
                    rclcpp::Time ros_time(out.header.stamp);

                    // ROS logger (recommended)
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Published MPEG frame | stamp = %.6f sec",
                        ros_time.seconds());
                    
                    
                gst_buffer_unmap(enc_buf, &enc_map);
            }

            gst_sample_unref(sample);
        }
    }

    // ---------- ROS ----------
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;

    // ---------- GStreamer ----------
    GstElement *pipeline_{nullptr};
    GstAppSrc *appsrc_{nullptr};
    GstAppSink *appsink_{nullptr};

    bool pipeline_ready_;
};

int main(int argc,char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPEGEncoderNode>());
    rclcpp::shutdown();
    return 0;
}
