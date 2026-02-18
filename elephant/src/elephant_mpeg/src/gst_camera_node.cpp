#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <sstream>
#include <iomanip>

class GstCameraNode : public rclcpp::Node
{
public:
    GstCameraNode() : Node("gst_camera_node"), frame_count_(0)
    {
        pub_ = create_publisher<sensor_msgs::msg::Image>(
            "camera/image_raw", 10);

        save_dir_ = "ros2_images";
        std::filesystem::create_directories(save_dir_);

        gst_init(nullptr, nullptr);

        const char *pipeline_desc =
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1 ! "
            "nvvidconv flip-method=2 ! "
            "video/x-raw,format=BGRx ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink name=sink max-buffers=1 drop=true";

        pipeline_ = gst_parse_launch(pipeline_desc, nullptr);

        appsink_ = GST_APP_SINK(
            gst_bin_get_by_name(GST_BIN(pipeline_), "sink"));

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        // ---- ROS timer pulls frames ----
        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&GstCameraNode::capture_frame, this));

        RCLCPP_INFO(get_logger(),"Camera pipeline started");
    }

    ~GstCameraNode()
    {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }

private:

    void capture_frame()
    {
        GstSample *sample =
            gst_app_sink_try_pull_sample(appsink_, 0);

        if(!sample)
            return;

        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstCaps *caps = gst_sample_get_caps(sample);
        GstStructure *s = gst_caps_get_structure(caps, 0);

        int width, height;
        gst_structure_get_int(s,"width",&width);
        gst_structure_get_int(s,"height",&height);

        GstMapInfo map;
        gst_buffer_map(buffer,&map,GST_MAP_READ);

        cv::Mat bgr(height,width,CV_8UC3,(void*)map.data);

        // Save image
        std::stringstream filename;
        filename << save_dir_
                 << "/frame_"
                 << std::setw(6)
                 << std::setfill('0')
                 << frame_count_
                 << ".png";

        cv::imwrite(filename.str(), bgr);

        // Publish
        sensor_msgs::msg::Image msg;
        msg.header.stamp = now();
        msg.width = width;
        msg.height = height;
        msg.encoding = "bgr8";
        msg.step = width * 3;
        msg.data.assign(map.data, map.data + map.size);

        pub_->publish(msg);

        frame_count_++;

        gst_buffer_unmap(buffer,&map);
        gst_sample_unref(sample);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    GstElement *pipeline_;
    GstAppSink *appsink_;

    std::string save_dir_;
    size_t frame_count_;
};
