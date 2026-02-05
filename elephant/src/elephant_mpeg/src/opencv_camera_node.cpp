#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class OpenCVCameraNode : public rclcpp::Node
{
public:
    OpenCVCameraNode() : Node("opencv_camera_node")
    {
        pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10);

        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_FATAL(get_logger(), "Failed to open camera");
            rclcpp::shutdown();
            return;
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap_.set(cv::CAP_PROP_FPS, 30);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&OpenCVCameraNode::captureFrame, this)
        );

        RCLCPP_INFO(get_logger(), "OpenCV camera node started");
    }

private:
    void captureFrame()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_WARN(get_logger(), "Empty frame captured");
            return;
        }

        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",
            frame
        ).toImageMsg();

        msg->header.stamp = now();
        msg->header.frame_id = "camera_link";

        pub_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OpenCVCameraNode>());
    rclcpp::shutdown();
    return 0;
}
