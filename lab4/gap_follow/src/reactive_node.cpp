#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries

#define MAX_RANGE_VALUE 2.0

using namespace std;

class ReactiveFollowGap : public rclcpp::Node
{
    // Implement Reactive Follow Gap on the car
    // This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&ReactiveFollowGap::scan_callback, this, std::placeholders::_1));

        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    // velocity 초기화
    float velocity = 0.0;

    vector<float> preprocess_lidar(vector<float> ranges)
    {
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

        int range_size = ranges.size();
        vector<float> proc_range(range_size); // preprocess ranges
        int window_size = 5;                  // 임시 지정 -> 어떻게 될 지 모름

        for (int i = 0; i < range_size; ++i)
        {
            int start = max(0, i - window_size / 2);
            int end = min(range_size, i + window_size / 2 + 1);

            float sum = 0.0;
            for (int j = start; j < end; ++j)
            {
                sum += ranges[j];
            }
            proc_range[i] = sum / (end - start);
        }

        // reject high value MAX_RANGE_VALUE
        for (int i = 0; i < range_size; ++i)
        {
            if (proc_range[i] > MAX_RANGE_VALUE)
            {
                proc_range[i] = MAX_RANGE_VALUE;
            }
        }

        return proc_range;
    }

    // 반환 start, end 2개 반환 pair이용
    pair<int, int> find_max_gap(vector<float> ranges)
    {
        // Return the start index & end index of the max gap in free_space_ranges

        int max_start = 0, max_end = 0, max_length = 0;
        int current_start = -1;

        for (int i = 0; i < ranges.size(); ++i)
        {
            if (ranges[i] > 0)
            {
                if (current_start == -1)
                {
                    current_start = i; // 정상출력부터 시작
                }
            }
            else
            {
                if (current_start != -1)
                {
                    int current_length = i - current_start;
                    // 계속해서 현재의 길이 파악하고, 현재 최대 길이와 비교하여 현재 길이가 최대 길이보다 길면 업데이트
                    // 그 시점이 최대 gap이고, 해당 index를 저장
                    if (current_length > max_length)
                    {
                        max_length = current_length;
                        max_start = current_start;
                        max_end = i;
                    }
                    current_start = -1;
                }
            }
        }

        // 마지막 gap 확인
        if (current_start != -1)
        {
            int current_length = ranges.size() - current_start;
            if (current_length > max_length)
            {
                max_length = current_length;
                max_start = current_start;
                max_end = ranges.size();
            }
        }

        return {max_start, max_end};
    }

    // range에서 가장 좋은 지점의 index return 이므로 반환타입은 int
    int find_best_point(vector<float> ranges, int start_i, int end_i)
    {
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
        // Naive: Choose the furthest point within ranges and go there
        int best_point = start_i;
        float max_gap = 0.0; // 최대 길이 gap이 좋은 것

        // start_i와 end_i 범위 내에서 가장 멀리 떨어진 point 찾기
        for (int i = start_i; i < end_i; ++i)
        {
            if (ranges[i] > max_gap)
            {
                max_gap = ranges[i];
                best_point = i;
            }
        }

        return best_point;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        // 데이터 preprocess
        vector<float> proc_range = preprocess_lidar(scan_msg->ranges);

        /// TODO:
        // Find closest point to LiDAR
        // 가장 가까운 점 찾기
        auto min_iter = min_element(proc_range.begin(), proc_range.end());
        int closet_point = distance(proc_range.begin(), min_iter);

        // Eliminate all points inside 'bubble' (set them to zero)
        // bubble 안의 모든 점을 제거를 하는데, 이 때 점은 0으로 설정
        int bubble_radius = 5; // window_size와 일치
        int start_bubble = max(0, closet_point - bubble_radius);
        int end_bubble = min((int)proc_range.size(), closet_point + bubble_radius);
        for (int i = start_bubble; i < end_bubble; ++i)
        {
            proc_range[i] = 0.0;
        }

        // Find max length gap
        // 가장 긴 gap 찾기 find_max_gap 이용
        auto [start_i, end_i] = find_max_gap(proc_range);

        // Find the best point in the gap
        // gap에서 가장 좋은 지점 찾기 find_best_point 이용
        int best_point = find_best_point(proc_range, start_i, end_i);

        // steering_angle을 best_point 기준의 angle 계산
        float angle = scan_msg->angle_min + best_point * scan_msg->angle_increment;
        // velocity 지정 wall_follow_node.cpp 에서 PID 제어에서 속도 지정 부분 참고
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

        // Publish Drive message
        // drive 메시지 publish
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.drive.speed = velocity;
        drive_msg.drive.steering_angle = angle;
        drive_publisher_->publish(drive_msg);
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}