#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <vector>
#include <cmath>
#include <limits>

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
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1));

        // subscrive 'ego_racecar/odom' topic, by f1tenth_gym
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&Safety::drive_callback, this, std::placeholders::_1));

        // publish '/drive/ topic, use AckermannDriveStamped
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    }

private:
    double speed = 0.0;
    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
    {
        /// TODO: update current speed
        speed = odom_msg->twist.twist.linear.x; // speed field in ac
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        /// TODO: calculate TTC
        float min_ttc = std::numeric_limits<float>::infinity(); // value init
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            float distance = scan_msg->ranges[i]; // 거리 확인
            if (distance > 0.0)                   // 물체에 박진 않아야 함
            {
                float angle = scan_msg->angle_min + i * scan_msg->angle_increment; // 라이다의 각도
                float relative_velocity = speed * std::cos(angle);                 // 상대 속도
                if (relative_velocity > 0)
                {
                    float ttc = distance / relative_velocity; // ttc 계산
                    if (ttc < min_ttc)
                    {
                        min_ttc = ttc; // 최소 ttc 계산
                    }
                }
            }
        }

        /// TODO: publish drive/brake message
        // TTC가 임계값보다 작으면 브레이크 메시지를 발행
        float ttc_threshold = 1.0;   // 임계값을 1초로 설정
        if (min_ttc < ttc_threshold) // 지정한 임계값보다 작으면 정지
        {
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.speed = 0.0;
            drive_publisher_->publish(drive_msg);
            RCLCPP_WARN(this->get_logger(), "Emergency brake activated! TTC is below threshold.");
        }
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}