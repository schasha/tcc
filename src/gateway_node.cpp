// gateway_node.cpp — ROS 2 sensors and actuators exported as SmartData.
//
// Every topic in config/sensors.yaml becomes a set of SmartData transducers
// (codegen/generate.py → parsed_transducers.hpp / parsed_gateway.hpp):
//
//   * sensors   — active transducers fed by the ROS subscription, each
//                 instance exported as an ADVERTISED Responsive SmartData. They
//                 answer interests from the SmartData network: on every
//                 update (event-driven) or every period (time-triggered).
//   * actuators — COMMANDED Responsive SmartData; a command from the network
//                 writes the transducer's field of the setpoint message and
//                 publishes it on the ROS topic.
//
// The node provides what the SD paper calls the run-time support: the network
// segment (UDP multicast, TSTP-style messages), the clock (ROS time, so
// simulated time works) and the location service (tf2, in target_frame).
//
// Optionally every update is mirrored on /smartdata/<topic> for inspection
// with the ROS tooling (enable_ros_mirror).

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2_ros/transform_listener.h"

#include "semantic_gateway/msg/smart_data.hpp"
#include "smartdata/network.hpp"
#include "parsed_gateway.hpp"

namespace sg = semantic_gateway;

class GatewayNode : public rclcpp::Node
{
public:
    GatewayNode() : Node("semantic_gateway")
    {
        target_frame_  = declare_parameter<std::string>("target_frame", "map");
        stats_period_s_ = declare_parameter<int>("stats_interval_s", 5);
        enable_mirror_ = declare_parameter<bool>("enable_ros_mirror", true);

        sd::Network::Config net;
        net.group  = declare_parameter<std::string>("bus_group", net.group);
        net.port   = static_cast<uint16_t>(declare_parameter<int>("bus_port", net.port));
        net.ifaddr = declare_parameter<std::string>("bus_ifaddr", "");

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        network_ = std::make_unique<sd::Network>(net, [this] {
            return static_cast<sd::Time>(get_clock()->now().nanoseconds() / 1000);
        });

        sg::gen::Update_Hook hook;
        if (enable_mirror_) {
            for (const char * topic : sg::gen::MIRROR_TOPICS) {
                mirrors_.emplace(topic, create_publisher<sg::msg::SmartData>(topic, rclcpp::QoS(200)));
            }
            hook = [this](const std::string & topic, const sd::Snapshot & s) { mirror(topic, s); };
        }

        gateway_ = std::make_unique<sg::gen::Gateway>(
            this, *network_,
            [this](const std::string & frame, const builtin_interfaces::msg::Time & stamp) {
                return locate(frame, stamp);
            },
            hook);

        for (const auto & t : sg::gen::TRANSDUCERS) {
            RCLCPP_INFO(get_logger(), "%-9s %-14s %-40s unit=%s dev=[%u, +%zu)",
                        t.actuator ? "actuator" : "sensor", t.topic, t.transducer,
                        si_units::to_string(t.unit).c_str(), t.dev_base, t.devices);
        }

        stats_timer_ = create_wall_timer(std::chrono::seconds(stats_period_s_),
                                         [this] { report_stats(); });

        RCLCPP_INFO(get_logger(),
            "gateway up. target_frame=%s, bus=%s:%u, ros_mirror=%s",
            target_frame_.c_str(), net.group.c_str(), net.port,
            enable_mirror_ ? "on" : "off");
    }

    ~GatewayNode() override
    {
        gateway_.reset();  // SmartData go before the network they are attached to
        network_.reset();
    }

private:
    // Location service: where `frame` was at `stamp`, in target_frame.
    // The confidence tells consumers whether the position can be trusted.
    sd::Origin locate(const std::string & frame, const builtin_interfaces::msg::Time & stamp)
    {
        sd::Origin o;
        if (frame.empty() || target_frame_.empty()) return o;
        try {
            const auto tf = tf_buffer_->lookupTransform(target_frame_, frame, tf2_ros::fromMsg(stamp));
            o.x = tf.transform.translation.x;
            o.y = tf.transform.translation.y;
            o.z = tf.transform.translation.z;
            o.confidence = 100;
            return o;
        } catch (const tf2::TransformException &) {
        }
        try {  // latest known transform: right place, possibly stale
            const auto tf = tf_buffer_->lookupTransform(target_frame_, frame, tf2::TimePointZero);
            o.x = tf.transform.translation.x;
            o.y = tf.transform.translation.y;
            o.z = tf.transform.translation.z;
            o.confidence = 50;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "TF %s -> %s unavailable: %s — location unknown",
                target_frame_.c_str(), frame.c_str(), ex.what());
        }
        return o;
    }

    void mirror(const std::string & topic, const sd::Snapshot & s)
    {
        auto it = mirrors_.find(topic);
        if (it == mirrors_.end()) return;
        sg::msg::SmartData m;
        m.unit          = s.unit;
        m.value         = s.reading.value;
        m.origin_x      = s.reading.origin.x;
        m.origin_y      = s.reading.origin.y;
        m.origin_z      = s.reading.origin.z;
        m.origin_device = s.device;
        m.origin_time   = static_cast<uint64_t>(s.reading.origin.t);
        m.expiry        = static_cast<uint64_t>(s.expiry);
        m.mac.fill(0);
        it->second->publish(m);
    }

    void report_stats()
    {
        RCLCPP_INFO(get_logger(), "stats: %zu SmartData, %lu datagrams sent, %lu received",
                    gateway_->smartdata(),
                    static_cast<unsigned long>(network_->sent()),
                    static_cast<unsigned long>(network_->received()));
    }

    std::string target_frame_;
    int         stats_period_s_{5};
    bool        enable_mirror_{true};

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unordered_map<std::string, rclcpp::Publisher<sg::msg::SmartData>::SharedPtr> mirrors_;

    std::unique_ptr<sd::Network>      network_;
    std::unique_ptr<sg::gen::Gateway> gateway_;
    rclcpp::TimerBase::SharedPtr      stats_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GatewayNode>());
    rclcpp::shutdown();
    return 0;
}
