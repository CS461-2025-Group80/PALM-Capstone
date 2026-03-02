
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fstream>
#include <iomanip>

class VelocityLogger : public rclcpp::Node
{
public:
    VelocityLogger()
    : Node("velocity_logger")
    {
        // Open CSV file
        csv_file_.open("velocity_log.csv", std::ios::out);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file!");
            return;
        }

        // Write CSV header
        csv_file_ << "timestamp,linear_x,linear_y,linear_z,angular_x,angular_y,angular_z\n";

        // Subscribe to odometry
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&VelocityLogger::odom_callback, this, std::placeholders::_1)
        );
    }

    ~VelocityLogger()
    {
        if (csv_file_.is_open())
            csv_file_.close();
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Get ROS timestamp in seconds
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        // Extract velocities
        double lx = msg->twist.twist.linear.x;
        double ly = msg->twist.twist.linear.y;
        double lz = msg->twist.twist.linear.z;
        double ax = msg->twist.twist.angular.x;
        double ay = msg->twist.twist.angular.y;
        double az = msg->twist.twist.angular.z;

        // Write to CSV
        csv_file_ << std::fixed << std::setprecision(6)
                  << timestamp << ","
                  << lx << "," << ly << "," << lz << ","
                  << ax << "," << ay << "," << az << "\n";

        RCLCPP_INFO(this->get_logger(),
                    "Logged -> t: %.6f, lin_x: %.2f, ang_z: %.2f",
                    timestamp, lx, az);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::ofstream csv_file_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityLogger>());
    rclcpp::shutdown();
    return 0;
}

