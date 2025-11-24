#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PoseSubscriber : public rclcpp::Node
{
public:
    PoseSubscriber() : Node("pose_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "vr_pose",              // Topic to subscribe to
            10,                     // QoS queue size
            std::bind(&PoseSubscriber::pose_callback, this, std::placeholders::_1)
        );
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Received Pose:\n"
            " Position -> x: %.2f, y: %.2f, z: %.2f\n"
            " Orientation -> x: %.2f, y: %.2f, z: %.2f, w: %.2f",
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseSubscriber>());
    rclcpp::shutdown();
    return 0;
}
