// validator_node.cpp — generic fidelity and integrity validator.
//
// Reads the same sensors.yaml as the gateway. For each spec:
//   * Subscribes to the original ROS topic via GenericSubscription and runs
//     the SAME decomposer on it — producing the "expected" SmartData.
//   * Subscribes to the configured SmartData output topic and matches every
//     incoming SmartData against the expected set, keyed by
//     (origin_time, unit, dev).
//
// Reported metrics (per topic and global):
//   * observed / matched / unmatched / expired counts
//   * scalar MSE between expected and observed values
//   * spatial RMSE between origin_{x,y,z} and a TF lookup at origin_time
// ──────────────────────────────────────────────────────────────────────────────

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
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

// Composite key: (unit, dev). Used to match expected and observed SmartData
// that share an origin_time.
struct UnitDev {
    uint32_t unit;
    uint32_t dev;
    bool operator==(const UnitDev & o) const { return unit == o.unit && dev == o.dev; }
};
struct UnitDevHash {
    std::size_t operator()(const UnitDev & k) const {
        return std::hash<uint64_t>{}(
            (static_cast<uint64_t>(k.unit) << 32) | k.dev);
    }
};

struct PerSensor {
    sg::SensorSpec spec;
    std::unique_ptr<sg::MessageView> view;
    std::mutex view_mtx;

    // origin_time_us → (unit,dev) → expected value
    std::unordered_map<uint64_t,
        std::unordered_map<UnitDev, double, UnitDevHash>> expected;
    std::mutex expected_mtx;

    rclcpp::SubscriptionBase::SharedPtr sub_raw;
    rclcpp::Subscription<sg::msg::SmartData>::SharedPtr sub_sd;

    std::size_t observed{0}, matched{0}, expired{0}, unmatched{0};
    double      sum_sq_err{0.0};
    double      spatial_sum_sq{0.0};
    std::size_t spatial_n{0};
};

}  // namespace

class ValidatorNode : public rclcpp::Node
{
public:
    ValidatorNode() : Node("smartdata_validator")
    {
        std::string config_path = declare_parameter<std::string>(
            "config_path",
            ament_index_cpp::get_package_share_directory("semantic_gateway")
                + "/config/sensors.yaml");
        report_period_s_ = declare_parameter<int>("report_interval_s", 5);
        buffer_horizon_us_ = static_cast<uint64_t>(
            declare_parameter<int>("buffer_horizon_ms", 2000)) * 1000ULL;

        const sg::GatewayConfig cfg = sg::load_config_file(config_path);
        target_frame_ = declare_parameter<std::string>(
            "target_frame", cfg.target_frame);

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        for (const auto & spec : cfg.sensors) {
            auto ps  = std::make_shared<PerSensor>();
            ps->spec = spec;
            ps->view = std::make_unique<sg::MessageView>(spec.type);
            PerSensor * raw = ps.get();

            // Original ROS sensor → populate expected[]
            ps->sub_raw = create_generic_subscription(
                spec.topic, spec.type, rclcpp::SensorDataQoS(),
                [this, raw](std::shared_ptr<rclcpp::SerializedMessage> m) {
                    on_raw(*raw, *m);
                });

            // SmartData → match against expected[]
            ps->sub_sd = create_subscription<sg::msg::SmartData>(
                spec.output_topic, rclcpp::QoS(200),
                [this, raw](sg::msg::SmartData::ConstSharedPtr m) {
                    on_sd(*raw, *m);
                });

            sensors_.push_back(ps);
            RCLCPP_INFO(get_logger(),
                "watching: %s ⇄ %s", spec.topic.c_str(), spec.output_topic.c_str());
        }

        report_timer_ = create_wall_timer(
            std::chrono::seconds(report_period_s_),
            [this] { report(); });
    }

private:
    void on_raw(PerSensor & ps, const rclcpp::SerializedMessage & serialized)
    {
        std::vector<sg::msg::SmartData> expected;
        {
            std::lock_guard<std::mutex> lock(ps.view_mtx);
            try {
                ps.view->deserialize(serialized);
                // Use (0,0,0) origin in the validator's expected: the spatial
                // diff is checked separately by TF lookup against what the
                // gateway actually emitted.
                expected = sg::decompose(ps.spec, *ps.view, 0.0, 0.0, 0.0);
            } catch (const std::exception & e) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "validator decompose failed on %s: %s",
                    ps.spec.topic.c_str(), e.what());
                return;
            }
        }
        if (expected.empty()) return;

        const uint64_t t = expected.front().origin_time;
        {
            std::lock_guard<std::mutex> lock(ps.expected_mtx);
            auto & by_key = ps.expected[t];
            for (const auto & sd : expected) {
                by_key[{sd.unit, sd.origin_device}] = sd.value;
            }
            gc(ps.expected, t);
        }
    }

    void on_sd(PerSensor & ps, const sg::msg::SmartData & sd)
    {
        ++ps.observed;
        const uint64_t now_us = static_cast<uint64_t>(
            get_clock()->now().nanoseconds() / 1000LL);
        if (!sg::is_valid(sd, now_us)) { ++ps.expired; return; }

        std::lock_guard<std::mutex> lock(ps.expected_mtx);
        auto it = ps.expected.find(sd.origin_time);
        if (it == ps.expected.end()) { ++ps.unmatched; return; }
        auto it2 = it->second.find({sd.unit, sd.origin_device});
        if (it2 == it->second.end()) { ++ps.unmatched; return; }

        const double d = it2->second - sd.value;
        ps.sum_sq_err += d * d;
        ++ps.matched;
        accumulate_spatial(ps, sd);
    }

    void accumulate_spatial(PerSensor & ps, const sg::msg::SmartData & sd)
    {
        std::string frame;
        try {
            std::lock_guard<std::mutex> lock(ps.view_mtx);
            frame = ps.view->read_string(ps.spec.frame_field);
        } catch (...) { return; }
        if (frame.empty()) return;

        builtin_interfaces::msg::Time stamp;
        stamp.sec     = static_cast<int32_t>(sd.origin_time / 1'000'000ULL);
        stamp.nanosec = static_cast<uint32_t>((sd.origin_time % 1'000'000ULL) * 1000ULL);
        try {
            const auto tf = tf_buffer_->lookupTransform(
                target_frame_, frame, tf2_ros::fromMsg(stamp));
            const double dx = tf.transform.translation.x - sd.origin_x;
            const double dy = tf.transform.translation.y - sd.origin_y;
            const double dz = tf.transform.translation.z - sd.origin_z;
            ps.spatial_sum_sq += dx*dx + dy*dy + dz*dz;
            ++ps.spatial_n;
        } catch (const tf2::TransformException &) {}
    }

    template <typename Map>
    void gc(Map & m, uint64_t newest)
    {
        if (m.size() < 256) return;
        const uint64_t cutoff =
            (newest > buffer_horizon_us_) ? newest - buffer_horizon_us_ : 0;
        for (auto it = m.begin(); it != m.end();) {
            it = (it->first < cutoff) ? m.erase(it) : std::next(it);
        }
    }

    void report()
    {
        RCLCPP_INFO(get_logger(), "──── Validation report ────");
        for (const auto & ps : sensors_) {
            const double mse = ps->matched
                ? ps->sum_sq_err / static_cast<double>(ps->matched) : 0.0;
            const double srmse = ps->spatial_n
                ? std::sqrt(ps->spatial_sum_sq / ps->spatial_n) : 0.0;
            RCLCPP_INFO(get_logger(),
                " %-22s  obs=%zu matched=%zu expired=%zu unmatched=%zu "
                "MSE=%.6g  spatial_RMSE=%.6f m",
                ps->spec.topic.c_str(),
                ps->observed, ps->matched, ps->expired, ps->unmatched,
                mse, srmse);
        }
    }

    std::string target_frame_;
    int         report_period_s_{5};
    uint64_t    buffer_horizon_us_{2'000'000ULL};

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<std::shared_ptr<PerSensor>> sensors_;
    rclcpp::TimerBase::SharedPtr            report_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ValidatorNode>());
    rclcpp::shutdown();
    return 0;
}
