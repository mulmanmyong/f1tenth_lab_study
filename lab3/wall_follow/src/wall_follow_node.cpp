#include <string>
#include <cmath>
#include <ctime>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std;

class WallFollow : public rclcpp::Node
{

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));

        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    }

private:
    // PID CONTROL PARAMS
    // TODO: double kp =
    // TODO: double kd =
    // TODO: double ki =

    // PID 파라미터 임의 지정
    float kp = 1.0;
    float kd = 0.01;
    float ki = 0.05;

    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;
    double derivative = 0.0;

    clock_t current_time = clock();

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers

    double get_range(vector<float> range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // TODO: implement

        const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg;
        int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;

        return range_data[index];
    }

    double get_error(vector<float> range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        // TODO:implement

        float angle_a = M_PI / 4.0; // 45 degress
        float angle_b = M_PI / 2.0; // 90 degrees

        // Laser Scan에서 a와 b의 각도에 따른 거리 반환
        float a = get_range(range_data, angle_a);
        float b = get_range(range_data, angle_b);

        // Laser Scan 사이의 각도 theta
        float theta = angle_b - angle_a;

        // 삼각법을 사용한 alpha, atan2
        float alpha = atan2(a * cosf(theta) - b, a * sinf(theta));

        // current distance D_t
        float D_t = b * cosf(alpha);

        // new distance, Look ahead distance L = 1.0이라고 가정
        float L = 1.0;
        float D_tp1 = D_t + L * sinf(alpha);

        // e(t) = desire distance - actual distance
        return dist - D_tp1;
    }

    void pid_control(double error, double velocity)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        double angle = 0.0;

        clock_t previous_time = current_time;
        current_time = clock();
        float dt = current_time - previous_time;

        integral = prev_error * dt;
        derivative = (error - prev_error) / dt;

        // TODO: Use kp, ki & kd to implement a PID controller
        angle = -((kp * error) + (ki * integral) + (kd * derivative));

        prev_error = error;
        if (abs(error) < 0.1)
        {
            angle = 0.0;
        }

        if (abs(angle) < M_PI / 180)
        {
            velocity = 1.5; // 0 < angle < 10
        }
        else if (abs(angle) < M_PI / 180)
        {
            velocity = 1.0; // 10 < angle < 20
        }
        else
        {
            velocity = 0.5; // else
        }

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish
        drive_msg.drive.speed = velocity;
        drive_msg.drive.steering_angle = angle;
        drive_publisher_->publish(drive_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        double error = get_error(scan_msg->ranges, 1.0); // TODO: replace with error calculated by get_error()
        double velocity = 0.0;                           // TODO: calculate desired car velocity based on error
        // TODO: actuate the car with PID
        pid_control(error, velocity);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}