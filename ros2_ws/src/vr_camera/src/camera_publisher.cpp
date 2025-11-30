#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>

class ImagePublisherNode : public rclcpp::Node {
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture camera_;
    cv::Mat frame_;
    int frame_count_;

public:
    ImagePublisherNode() : Node("image_publisher_node"), frame_count_(0) {
        // Declare parameters
        this->declare_parameter("camera_id", 0);
        this->declare_parameter("publish_rate", 30.0);
        this->declare_parameter("frame_width", 640);
        this->declare_parameter("frame_height", 480);
        this->declare_parameter("topic_name", "camera/image_raw");

        // Get parameters
        int camera_id = this->get_parameter("camera_id").as_int();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        int frame_width = this->get_parameter("frame_width").as_int();
        int frame_height = this->get_parameter("frame_height").as_int();
        std::string topic_name = this->get_parameter("topic_name").as_string();

        // Initialize camera
        camera_.open(camera_id);
        if (!camera_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera %d", camera_id);
            throw std::runtime_error("Camera initialization failed");
        }

        // Set camera properties
        camera_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
        camera_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);

        RCLCPP_INFO(this->get_logger(), "Camera opened successfully: %dx%d @ %.1f Hz",
                    frame_width, frame_height, publish_rate);

        // Create publisher
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);

        // Create timer for periodic publishing
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&ImagePublisherNode::publishImage, this)
        );

        RCLCPP_INFO(this->get_logger(), "Publishing images to topic: %s", topic_name.c_str());
    }

    ~ImagePublisherNode() {
        if (camera_.isOpened()) {
            camera_.release();
            RCLCPP_INFO(this->get_logger(), "Camera released");
        }
    }

private:
    void publishImage() {
        // Capture frame
        if (!camera_.read(frame_)) {
            RCLCPP_WARN(this->get_logger(), "Failed to capture frame");
            return;
        }

        // Convert OpenCV Mat to ROS2 Image message
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_frame";

        try {
            // Convert BGR to RGB (ROS convention)
            cv::Mat rgb_frame;
            cv::cvtColor(frame_, rgb_frame, cv::COLOR_BGR2RGB);

            // Create cv_bridge image
            auto cv_image = std::make_shared<cv_bridge::CvImage>(header, "rgb8", rgb_frame);
            
            // Convert to ROS message and publish
            auto msg = cv_image->toImageMsg();
            image_publisher_->publish(*msg);

            frame_count_++;
            if (frame_count_ % 30 == 0) {
                RCLCPP_INFO(this->get_logger(), "Published %d frames", frame_count_);
            }
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<ImagePublisherNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}

// Run with parameters:
// ros2 run <package_name> image_publisher_node --ros-args -p camera_id:=0 -p publish_rate:=30.0

// View the published images:
// ros2 run rqt_image_view rqt_image_view