// gateway_node.cpp — generic semantic gateway: ROS sensor topics → SmartData
//
// Codegen path (see codegen/generate.py and config/sensors.yaml):
//   * config/sensors.yaml is the BUILD-time spec; it is no longer parsed at
//     runtime. CMake re-runs the generator when the YAML changes.
//   * For each sensor in the YAML the generator emits a typed function
//     decompose_<topic>(msg, ox, oy, oz, out). The dispatch header wires a
//     rclcpp::Subscription<T> per sensor calling that function.
//
// Output path:
//   * Primary: UDP multicast bus (see docs/wire_format.md). Other SmartData
//     systems (SDAV transducers etc.) consume this directly.
//   * Optional mirror on ROS topics for debug/observability — controlled by
//     the `enable_ros_mirror` parameter (default true).

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2/exceptions.h"

#include "semantic_gateway/msg/smart_data.hpp"

#include "parsed_dispatch.hpp"
#include "sd_bus.hpp"

namespace sg = semantic_gateway;

class GatewayNode : public rclcpp::Node
{
public:
    GatewayNode() : Node("semantic_gateway")
    {
        target_frame_   = declare_parameter<std::string>("target_frame", "map");
        stats_period_s_ = declare_parameter<int>        ("stats_interval_s", 5);

        enable_ros_mirror_ = declare_parameter<bool>("enable_ros_mirror", true);
        bus_group_         = declare_parameter<std::string>("bus_group", sg::SDBus::DEFAULT_GROUP);
        bus_port_          = declare_parameter<int>        ("bus_port",  sg::SDBus::DEFAULT_PORT);
        bus_ifaddr_        = declare_parameter<std::string>("bus_ifaddr", "");

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        bus_ = std::make_unique<sg::SDBus>(
            bus_group_, static_cast<uint16_t>(bus_port_), bus_ifaddr_);

        auto lookup = [this](const std::string & frame,
                              const builtin_interfaces::msg::Time & stamp)
            -> std::tuple<double,double,double> {
            return lookup_origin(frame, stamp);
        };

        auto on_sd = [this](const std::string & out_topic,
                             const sg::msg::SmartData & sd) {
            bus_->broadcast(sd);
            ++sd_counts_[out_topic];

            if (enable_ros_mirror_) {
                auto it = pubs_.find(out_topic);
                if (it == pubs_.end()) {
                    it = pubs_.emplace(out_topic,
                        create_publisher<sg::msg::SmartData>(
                            out_topic, rclcpp::QoS(200))).first;
                }
                it->second->publish(sd);
            }
        };

        subs_ = sg::gen::register_all(this, lookup, on_sd);

        stats_timer_ = create_wall_timer(
            std::chrono::seconds(stats_period_s_),
            [this] { report_stats(); });

        RCLCPP_INFO(get_logger(),
            "gateway up. target_frame=%s, %zu sensors, bus=%s:%d, ros_mirror=%s",
            target_frame_.c_str(), subs_.handles.size(),
            bus_group_.c_str(), bus_port_,
            enable_ros_mirror_ ? "on" : "off");
    }

private:
    std::tuple<double,double,double> lookup_origin(
        const std::string & sensor_frame,
        const builtin_interfaces::msg::Time & stamp)
    {
        if (sensor_frame.empty() || target_frame_.empty()) return {0.0, 0.0, 0.0};
        try {
            const auto tf = tf_buffer_->lookupTransform(
                target_frame_, sensor_frame, tf2_ros::fromMsg(stamp));
            return { tf.transform.translation.x,
                     tf.transform.translation.y,
                     tf.transform.translation.z };
        } catch (const tf2::TransformException &) {
            try {
                const auto tf = tf_buffer_->lookupTransform(
                    target_frame_, sensor_frame, tf2::TimePointZero);
                return { tf.transform.translation.x,
                         tf.transform.translation.y,
                         tf.transform.translation.z };
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "TF %s → %s unavailable: %s — origin (0,0,0)",
                    target_frame_.c_str(), sensor_frame.c_str(), ex.what());
                return {0.0, 0.0, 0.0};
            }
        }
    }

    void report_stats()
    {
        for (const auto & [topic, count] : sd_counts_) {
            RCLCPP_INFO(get_logger(), "stats: %-26s %zu SD broadcast",
                        topic.c_str(), count);
        }
    }

    std::string target_frame_;
    int         stats_period_s_{5};

    bool        enable_ros_mirror_{true};
    std::string bus_group_;
    int         bus_port_{0};
    std::string bus_ifaddr_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unique_ptr<sg::SDBus> bus_;

    sg::gen::Subs subs_;
    std::unordered_map<std::string, rclcpp::Publisher<sg::msg::SmartData>::SharedPtr> pubs_;
    std::unordered_map<std::string, std::size_t> sd_counts_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GatewayNode>());
    rclcpp::shutdown();
    return 0;
}
