#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class Talker : public rclcpp::Node
{
public:
    Talker() : Node("talker")
    {
        this->declare_parameter<double>("v", 1.0);
        this->declare_parameter<double>("d", 0.0);
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Talker::publish_message, this));
    }

private:
    void publish_message()
    {
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        msg.drive.speed = this->get_parameter("v").as_double();
        msg.drive.steering_angle = this->get_parameter("d").as_double();
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing: speed=%.2f, steering_angle=%.2f", msg.drive.speed, msg.drive.steering_angle);
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Talker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
