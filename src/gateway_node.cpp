// gateway_node.cpp — generic semantic gateway: ROS sensor topics → SmartData
//
// for each spec in the loaded config the node creates:
//   * a rclcpp::GenericSubscription on the input ROS topic (bytes in)
//   * a Publisher<SmartData> on the configured output topic
//   * a per-spec MessageView that deserializes those bytes via introspection
//
// on each message, the gateway does TF lookup for the spec's sensor frame
// (read from the message via `frame_field`) and calls decompose(...)

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2/exceptions.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "config_loader.hpp"
#include "decomposer.hpp"
#include "field_access.hpp"
#include "semantic_gateway/msg/smart_data.hpp"
#include "si_units.hpp"
#include "spec.hpp"

namespace sg = semantic_gateway;

namespace {
struct Origin3D { double x{0.0}, y{0.0}, z{0.0}; };

// per-sensor runtime state.
struct SensorPipe {
    sg::SensorSpec                                                  spec;
    rclcpp::Publisher<sg::msg::SmartData>::SharedPtr                pub;
    std::shared_ptr<rclcpp::GenericSubscription>                    sub;
    std::unique_ptr<sg::MessageView>                                view;
    std::mutex                                                      view_mtx;
    std::size_t                                                     msgs{0};
    std::size_t                                                     sd_count{0};
};
}  // namespace

class GatewayNode : public rclcpp::Node
{
public:
    GatewayNode() : Node("semantic_gateway")
    {
        std::string config_path = declare_parameter<std::string>(
            "config_path",
            ament_index_cpp::get_package_share_directory("semantic_gateway")
                + "/config/sensors.yaml");
        stats_period_s_ = declare_parameter<int>("stats_interval_s", 5);

        sg::GatewayConfig cfg = sg::load_config_file(config_path);
        target_frame_ = declare_parameter<std::string>(
            "target_frame", cfg.target_frame);

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        for (const auto & spec : cfg.sensors) {
            auto pipe   = std::make_shared<SensorPipe>();
            pipe->spec  = spec;
            pipe->view  = std::make_unique<sg::MessageView>(spec.type);
            pipe->pub   = create_publisher<sg::msg::SmartData>(
                spec.output_topic, rclcpp::QoS(200));
            // capture raw pointer; SensorPipe lifetime is bound to pipes_
            SensorPipe * p = pipe.get();
            pipe->sub = create_generic_subscription(
                spec.topic, spec.type, rclcpp::SensorDataQoS(),
                [this, p](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                    handle(*p, *msg);
                });
            pipes_.push_back(pipe);
            RCLCPP_INFO(get_logger(),
                "registered sensor: topic=%s type=%s decomposition=%s out=%s",
                spec.topic.c_str(), spec.type.c_str(),
                spec.decomposition == sg::Decomposition::Scalar    ? "scalar"
              : spec.decomposition == sg::Decomposition::Array     ? "array"
                                                                    : "composite",
                spec.output_topic.c_str());
        }

        stats_timer_ = create_wall_timer(
            std::chrono::seconds(stats_period_s_),
            [this] { report_stats(); });

        RCLCPP_INFO(get_logger(),
            "gateway up. target_frame=%s, %zu sensors from %s",
            target_frame_.c_str(), pipes_.size(), config_path.c_str());
    }

private:
    void handle(SensorPipe & pipe, const rclcpp::SerializedMessage & serialized)
    {
        std::lock_guard<std::mutex> lock(pipe.view_mtx);
        try {
            pipe.view->deserialize(serialized);
        } catch (const std::exception & e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "deserialize failed for %s: %s",
                pipe.spec.topic.c_str(), e.what());
            return;
        }

        std::string frame;
        try { frame = pipe.view->read_string(pipe.spec.frame_field); }
        catch (const std::exception & e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "frame_field '%s' unreadable on %s: %s",
                pipe.spec.frame_field.c_str(), pipe.spec.topic.c_str(), e.what());
        }

        builtin_interfaces::msg::Time stamp_msg;
        try {
            const uint64_t t_us = pipe.view->read_time_us(pipe.spec.stamp_field);
            stamp_msg.sec     = static_cast<int32_t>(t_us / 1'000'000ULL);
            stamp_msg.nanosec = static_cast<uint32_t>((t_us % 1'000'000ULL) * 1000ULL);
        } catch (const std::exception &) {}

        const Origin3D o = lookup_origin(frame, stamp_msg);

        std::vector<sg::msg::SmartData> sds;
        try {
            sds = sg::decompose(pipe.spec, *pipe.view, o.x, o.y, o.z);
        } catch (const std::exception & e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "decompose failed for %s: %s",
                pipe.spec.topic.c_str(), e.what());
            return;
        }

        for (const auto & sd : sds) pipe.pub->publish(sd);
        ++pipe.msgs;
        pipe.sd_count += sds.size();
    }

    Origin3D lookup_origin(const std::string & sensor_frame,
                            const builtin_interfaces::msg::Time & stamp)
    {
        if (sensor_frame.empty() || target_frame_.empty()) return {};
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
                return {};
            }
        }
    }

    void report_stats()
    {
        for (const auto & p : pipes_) {
            RCLCPP_INFO(get_logger(),
                "stats: %-26s %zu msgs → %zu SD",
                p->spec.topic.c_str(), p->msgs, p->sd_count);
        }
    }

    std::string target_frame_;
    int         stats_period_s_{5};

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<std::shared_ptr<SensorPipe>> pipes_;
    rclcpp::TimerBase::SharedPtr             stats_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GatewayNode>());
    rclcpp::shutdown();
    return 0;
}
