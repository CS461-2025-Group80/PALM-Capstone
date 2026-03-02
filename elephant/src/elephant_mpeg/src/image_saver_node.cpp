#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <filesystem>
#include <string>
#include <sstream>
#include <iomanip>

namespace fs = std::filesystem;

class ImageSaverNode : public rclcpp::Node
{
public:
  ImageSaverNode()
  : Node("image_saver"), frame_count_(0), saved_count_(0)
  {
    // Declare parameters
    this->declare_parameter<std::string>("output_dir", "/saved_images");
    this->declare_parameter<std::string>("image_format", "png");
    this->declare_parameter<int>("save_every_n_frames", 1);

    output_dir_    = this->get_parameter("output_dir").as_string();
    image_format_  = this->get_parameter("image_format").as_string();
    save_every_n_  = this->get_parameter("save_every_n_frames").as_int();
    RCLCPP_INFO(this->get_logger(), "Check 1");
    // Create output directory
    RCLCPP_INFO(this->get_logger(), "Saving images to: %s", output_dir_.c_str());

        RCLCPP_INFO(this->get_logger(), "Check 2");

    // Match publisher QoS: RELIABLE + VOLATILE
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/image_raw",
      qos,
      std::bind(&ImageSaverNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Image saver node started. Waiting for images...");
  }

  ~ImageSaverNode()
  {
    RCLCPP_INFO(this->get_logger(),
      "Shutting down. Saved %zu images to %s", saved_count_, output_dir_.c_str());
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    ++frame_count_;

    if (frame_count_ % save_every_n_ != 0) {
      return;
    }

    // Convert ROS image to OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Build filename: frame_000001_<sec>_<nanosec>.<format>
    std::ostringstream oss;
    oss << output_dir_ << "/frame_"
        << std::setw(6) << std::setfill('0') << saved_count_ << "_"
        << msg->header.stamp.sec << "_"
        << std::setw(9) << std::setfill('0') << msg->header.stamp.nanosec
        << "." << image_format_;

    const std::string filepath = oss.str();

    if (!cv::imwrite(filepath, cv_ptr->image)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write image: %s", filepath.c_str());
      return;
    }

    ++saved_count_;

    // Throttle log to once every 2 seconds
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Saved image %zu: frame_%06zu (%ux%u)",
      saved_count_, saved_count_ - 1, msg->width, msg->height);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

  std::string output_dir_;
  std::string image_format_;
  int         save_every_n_;
  size_t      frame_count_;
  size_t      saved_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSaverNode>());
  rclcpp::shutdown();
  return 0;
}