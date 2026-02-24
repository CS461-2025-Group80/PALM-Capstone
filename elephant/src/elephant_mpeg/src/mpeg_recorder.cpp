#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

class ImageToMpegNode : public rclcpp::Node {
public:
    ImageToMpegNode() : Node("mpeg_recorder_node") {
        // 1. Parameters for flexibility
        this->declare_parameter("output_path", "output_video.mp4");
        this->declare_parameter("fps", 30.0);

        std::string output_path = this->get_parameter("output_path").as_string();
        double fps = this->get_parameter("fps").as_double();

        // 2. Subscribe to gscam output (usually /image_raw)
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, 
            std::bind(&ImageToMpegNode::image_callback, this, std::placeholders::_1));

        output_path_ = output_path;
        fps_ = fps;
        RCLCPP_INFO(this->get_logger(), "Saving video to: %s", output_path_.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS image to OpenCV Mat
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // Initialize VideoWriter on the first frame received
            if (!video_writer_.isOpened()) {
                int fourcc = cv::VideoWriter::fourcc('M', 'P', '4', 'V'); // MPEG-4 codec
                video_writer_.open(output_path_, fourcc, fps_, frame.size(), true);
                
                if (!video_writer_.isOpened()) {
                    RCLCPP_ERROR(this->get_logger(), "Could not open video file for writing.");
                    return;
                }
            }

            video_writer_.write(frame);
            RCLCPP_INFO_ONCE(this->get_logger(), "Recording started...");

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::VideoWriter video_writer_;
    std::string output_path_;
    double fps_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageToMpegNode>());
    rclcpp::shutdown();
    return 0;
}