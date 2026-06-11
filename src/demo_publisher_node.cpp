// demo_publisher_node.cpp — sensor publisher for gateway testing
//
// publishes realistic-looking LaserScan, Imu, and Temperature messages on the
// standard ROS 2 topics, plus a static TF2 transform, so the gateway_node can
// be exercised without a real robot or Gazebo simulation.
//
// sensor simulation:
//   - LaserScan: 360-beam scan with obstacle at varying distance
//   - Imu: gravity + small random noise on all axes
//   - Temperature: slow drift around 25 C
//
// also broadcasts a static transform:  map → base_link → laser_frame / imu_frame

#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

class DemoPublisherNode : public rclcpp::Node
{
public:
    explicit DemoPublisherNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
    : Node("demo_publisher", opts), rng_(42), noise_(0.0, 0.05)
    {
        // publishers
        laser_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan",        10);
        imu_pub_   = create_publisher<sensor_msgs::msg::Imu>       ("/imu/data",   10);
        temp_pub_  = create_publisher<sensor_msgs::msg::Temperature>("/temperature", 10);

        tf_static_bc_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        broadcast_static_transforms();

        // timers
        laser_timer_ = create_wall_timer(100ms, [this]{ publish_laser(); });    // 10 Hz
        imu_timer_   = create_wall_timer( 10ms, [this]{ publish_imu(); });      // 100 Hz
        temp_timer_  = create_wall_timer(500ms, [this]{ publish_temperature(); }); // 2 Hz

        RCLCPP_INFO(get_logger(),
            "DemoPublisher started. Generating synthetic sensor data.");
    }

private:
    std::mt19937                         rng_;
    std::normal_distribution<double>     noise_;

    // publishers
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr   laser_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr         imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_bc_;

    // timers
    rclcpp::TimerBase::SharedPtr laser_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::TimerBase::SharedPtr temp_timer_;

    // state
    double t_{0.0};   // simulation time in seconds (advanced by timers)

    void broadcast_static_transforms()
    {
        std::vector<geometry_msgs::msg::TransformStamped> tfs(3);

        // map -> base_link  (robot moves in the map frame; here it's static for demo)
        auto & tf0 = tfs[0];
        tf0.header.stamp       = now();
        tf0.header.frame_id    = "map";
        tf0.child_frame_id     = "base_link";
        tf0.transform.translation.x = 1.0;
        tf0.transform.translation.y = 2.0;
        tf0.transform.translation.z = 0.0;
        tf0.transform.rotation.w    = 1.0;

        // base_link -> laser_frame  (laser mounted at front, 0.2 m forward)
        auto & tf1 = tfs[1];
        tf1.header.stamp       = now();
        tf1.header.frame_id    = "base_link";
        tf1.child_frame_id     = "laser_frame";
        tf1.transform.translation.x = 0.2;
        tf1.transform.rotation.w    = 1.0;

        // base_link -> imu_frame  (IMU at robot center)
        auto & tf2 = tfs[2];
        tf2.header.stamp       = now();
        tf2.header.frame_id    = "base_link";
        tf2.child_frame_id     = "imu_frame";
        tf2.transform.rotation.w = 1.0;

        tf_static_bc_->sendTransform(tfs);
        RCLCPP_INFO(get_logger(), "Static TF broadcast: map→base_link→{laser_frame,imu_frame}");
    }

    void publish_laser()
    {
        t_ += 0.1;
        constexpr int N = 360;

        auto msg = sensor_msgs::msg::LaserScan();
        msg.header.stamp    = now();
        msg.header.frame_id = "laser_frame";
        msg.angle_min       = -static_cast<float>(M_PI);
        msg.angle_max       =  static_cast<float>(M_PI);
        msg.angle_increment = 2.0f * static_cast<float>(M_PI) / N;
        msg.time_increment  = 0.0f;
        msg.scan_time       = 0.1f;
        msg.range_min       = 0.1f;
        msg.range_max       = 30.0f;
        msg.ranges.resize(N);

        // simulated obstacle: a box at varying distance
        for (int i = 0; i < N; ++i) {
            float angle = msg.angle_min + i * msg.angle_increment;
            // simulate a wall at ~3 m plus sin(x) variance
            float r = 3.0f + 0.5f * std::sin(static_cast<float>(t_) + angle);
            r += static_cast<float>(noise_(rng_));
            msg.ranges[i] = std::max(msg.range_min, std::min(msg.range_max, r));
        }

        laser_pub_->publish(msg);
    }

    void publish_imu()
    {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp    = now();
        msg.header.frame_id = "imu_frame";

        // orientation: identity (no rotation)
        msg.orientation.w = 1.0;
        msg.orientation_covariance[0] = 0.001;
        msg.orientation_covariance[4] = 0.001;
        msg.orientation_covariance[8] = 0.001;

        // linear acceleration: gravity on Z + noise
        msg.linear_acceleration.x = noise_(rng_) * 0.1;
        msg.linear_acceleration.y = noise_(rng_) * 0.1;
        msg.linear_acceleration.z = 9.81 + noise_(rng_) * 0.05;
        msg.linear_acceleration_covariance[0] = 0.01;
        msg.linear_acceleration_covariance[4] = 0.01;
        msg.linear_acceleration_covariance[8] = 0.01;

        // angular velocity: small noise (robot approximately stationary)
        msg.angular_velocity.x = noise_(rng_) * 0.01;
        msg.angular_velocity.y = noise_(rng_) * 0.01;
        msg.angular_velocity.z = noise_(rng_) * 0.01;
        msg.angular_velocity_covariance[0] = 0.001;
        msg.angular_velocity_covariance[4] = 0.001;
        msg.angular_velocity_covariance[8] = 0.001;

        imu_pub_->publish(msg);
    }

    void publish_temperature()
    {
        auto msg = sensor_msgs::msg::Temperature();
        msg.header.stamp    = now();
        msg.header.frame_id = "base_link";

        // ambient temperature with slow 24-hour variation simulated at fast rate
        msg.temperature = 25.0 + 3.0 * std::sin(t_ * 0.01);
        msg.variance    = 0.04;  // 0.2 C stddev

        temp_pub_->publish(msg);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemoPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
