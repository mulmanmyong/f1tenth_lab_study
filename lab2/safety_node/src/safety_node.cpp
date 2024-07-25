#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class Safety : public rclcpp::Node
{
    // The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers

        // subscribe '/scan' topic, use lase_scan message
        scan_subscriber_ = this->create_subscription < sensor_msgs::msg::LaserScan("/scan", 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1));

        // subscrive 'ego_racecar/odom' topic, by f1tenth_gym
        odom_subscriber_ = this->create_subscription < senson_msgs::msg::Odometry("/ego_racecar/odom", 10, std::bind(&safety::drive_callback, this, std::placeholders::_1));

        // publish '/drive/ topic, use AckermannDriveStamped
        publisher_ = this->create_publisher < ackermann_msgs::msg::AckermannDriveStamped("/drive", 10);
    }

private:
    double speed = 0.0;
    /// TODO: create ROS subscribers and publishers

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        speed = msg->twist.twist.linear.x; // speed field in ac
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        /// TODO: calculate TTC
        float angle_min = scan_msg->angle_min; // angle_min to angle increment
        float angle_increment = scan_msg->angle_increment;
        const std::vector<float> &ranges = scan_msg->ranges;

        for (int i = 0; i < ranges.size(); i++)
        {
        }

        /// TODO: publish drive/brake message
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}