#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PosePublisher : public rclcpp::Node
{
public:
    PosePublisher() : Node("pose_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("vr_pose", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&PosePublisher::publish_pose, this)
        );
    }

private:
    void publish_pose()
    {
        geometry_msgs::msg::PoseStamped msg;

        // Timestamp + frame
        msg.header.stamp = this->now();
        msg.header.frame_id = "camera_link";

        // Position (example values)
        msg.pose.position.x = 1.0;
        msg.pose.position.y = 2.0;
        msg.pose.position.z = 3.0;

        // Orientation quaternion (example values)
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 1.0;

        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PosePublisher>());
    rclcpp::shutdown();
    return 0;
}
