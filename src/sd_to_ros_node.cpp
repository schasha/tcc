// sd_to_ros_node.cpp — inverse leg of the gateway.
//
// Listens on the SmartData multicast bus (see docs/wire_format.md) and
// reassembles each batch into the original ROS message type, republishing
// it on the recovered topic (default: <input>_recovered).
//
// Per-sensor state, batching and dispatch are wired by the codegen:
//   * parsed_recompose.hpp     feed_<name> / flush_<name> per sensor
//   * parsed_sd_dispatch.hpp   Consumers + dispatch_sd / drain
//
// Batching policy: a batch is the set of SmartData sharing the same
// origin_time for a sensor. dispatch flushes a sensor's bucket when a new
// origin_time arrives; the drain timer flushes any remainder periodically.

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "semantic_gateway/msg/smart_data.hpp"

#include "sd_bus.hpp"
#include "parsed_sd_dispatch.hpp"

namespace sg = semantic_gateway;
using namespace std::chrono_literals;

class SdToRosNode : public rclcpp::Node
{
public:
    SdToRosNode() : Node("sd_to_ros_node")
    {
        bus_group_   = declare_parameter<std::string>("bus_group",  sg::SDBus::DEFAULT_GROUP);
        bus_port_    = declare_parameter<int>        ("bus_port",   sg::SDBus::DEFAULT_PORT);
        bus_ifaddr_  = declare_parameter<std::string>("bus_ifaddr", "");
        drain_ms_    = declare_parameter<int>        ("drain_period_ms", 50);

        // recovered topics — keep aligned with codegen sensor names
        pub_scan_  = create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan_recovered", rclcpp::SensorDataQoS());
        pub_imu_   = create_publisher<sensor_msgs::msg::Imu>(
            "/imu/data_recovered", rclcpp::SensorDataQoS());
        pub_temp_  = create_publisher<sensor_msgs::msg::Temperature>(
            "/temperature_recovered", 10);

        consumers_.on_scan = [this](const sensor_msgs::msg::LaserScan & m) {
            pub_scan_->publish(m); ++scans_;
        };
        consumers_.on_imu_data = [this](const sensor_msgs::msg::Imu & m) {
            pub_imu_->publish(m); ++imus_;
        };
        consumers_.on_temperature = [this](const sensor_msgs::msg::Temperature & m) {
            pub_temp_->publish(m); ++temps_;
        };

        bus_ = std::make_unique<sg::SDBus>(
            bus_group_, static_cast<uint16_t>(bus_port_), bus_ifaddr_);
        bus_->listen([this](const sg::msg::SmartData & sd) {
            std::lock_guard<std::mutex> lk(mu_);
            sg::gen::dispatch_sd(consumers_, sd);
        });

        drain_timer_ = create_wall_timer(
            std::chrono::milliseconds(drain_ms_),
            [this] {
                std::lock_guard<std::mutex> lk(mu_);
                sg::gen::drain(consumers_);
            });

        RCLCPP_INFO(get_logger(),
            "sd_to_ros up. bus=%s:%d, drain=%dms",
            bus_group_.c_str(), bus_port_, drain_ms_);
    }

    ~SdToRosNode() override
    {
        if (bus_) bus_->stop();
    }

private:
    std::string bus_group_;
    int         bus_port_{0};
    std::string bus_ifaddr_;
    int         drain_ms_{50};

    std::unique_ptr<sg::SDBus> bus_;
    sg::gen::Consumers consumers_;
    std::mutex mu_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr   pub_scan_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr         pub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp_;
    rclcpp::TimerBase::SharedPtr drain_timer_;

    std::size_t scans_{0}, imus_{0}, temps_{0};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SdToRosNode>());
    rclcpp::shutdown();
    return 0;
}
