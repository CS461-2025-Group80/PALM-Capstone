#include <memory>
#include <string>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

namespace fs = std::filesystem;

class SegmentedMpegNode : public rclcpp::Node {
public:
    SegmentedMpegNode() : Node("segmented_recorder_node"), first_frame_(true) {
        // Parameters
        this->declare_parameter("output_dir", "videos");
        this->declare_parameter("segment_duration", 10.0); // Seconds per file
        this->declare_parameter("fps", 30.0);

        output_dir_ = this->get_parameter("output_dir").as_string();
        segment_duration_ = this->get_parameter("segment_duration").as_double();
        fps_ = this->get_parameter("fps").as_double();

        if (!fs::exists(output_dir_)) {
            fs::create_directories(output_dir_);
        }

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, 
            std::bind(&SegmentedMpegNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Recording started. Segments every %.1f seconds.", segment_duration_);
    }

private:
    void start_new_segment(const rclcpp::Time& stamp, const cv::Size& size) {
        if (video_writer_.isOpened()) {
            video_writer_.release();
        }

        // Generate filename: vid_1708713356.456.mp4
        char filename[128];
        snprintf(filename, sizeof(filename), "vid_%ld.%09ld.mp4", stamp.nanoseconds() / 1000000000, stamp.nanoseconds() % 1000000000);
        
        fs::path full_path = fs::path(output_dir_) / filename;
        int fourcc = cv::VideoWriter::fourcc('M', 'P', '4', 'V');
        
        video_writer_.open(full_path.string(), fourcc, fps_, size, true);
        last_segment_start_ = stamp;
        
        RCLCPP_INFO(this->get_logger(), "Started new segment: %s", filename);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            rclcpp::Time current_stamp = msg->header.stamp;

            // Check if we need to start a new file
            bool time_to_rotate = (current_stamp - last_segment_start_).seconds() >= segment_duration_;
            
            if (first_frame_ || time_to_rotate) {
                start_new_segment(current_stamp, frame.size());
                first_frame_ = false;
            }

            video_writer_.write(frame);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::VideoWriter video_writer_;
    std::string output_dir_;
    double segment_duration_;
    double fps_;
    rclcpp::Time last_segment_start_;
    bool first_frame_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SegmentedMpegNode>());
    rclcpp::shutdown();
    return 0;
}