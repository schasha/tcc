// test_gateway_pipeline.cpp — integration test for the generic pipeline.
//
// loads a config from a YAML string, simulates one ROS message per spec by
// serializing a sensor_msgs/Temperature, sensor_msgs/Imu, sensor_msgs/LaserScan
// and exercises: GenericSubscription -> MessageView ->  decompose -> SmartData
// in the same node

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "config_loader.hpp"
#include "decomposer.hpp"
#include "field_access.hpp"
#include "semantic_gateway/msg/smart_data.hpp"
#include "si_units.hpp"

using namespace std::chrono_literals;
namespace sg = semantic_gateway;

namespace {

constexpr int LASER_BEAMS = 100;

const char * CONFIG = R"(
defaults:
  target_frame: map
sensors:
  - topic: /scan
    type: sensor_msgs/msg/LaserScan
    decomposition: array
    array_field: ranges
    unit: meter
    filter: { kind: finite_within, min_field: range_min, max_field: range_max }
    expiry_ms: 100
  - topic: /imu/data
    type: sensor_msgs/msg/Imu
    decomposition: composite
    groups:
      - unit: meter_per_sec_sq
        fields: [linear_acceleration.x, linear_acceleration.y, linear_acceleration.z]
      - unit: rad_per_sec
        fields: [angular_velocity.x, angular_velocity.y, angular_velocity.z]
    expiry_ms: 10
  - topic: /temperature
    type: sensor_msgs/msg/Temperature
    decomposition: scalar
    value_field: temperature
    value_transform: { kind: linear, a: 1.0, b: 273.15 }
    unit: kelvin
    expiry_ms: 500
)";

// in-process generic gateway, modelled on src/gateway_node.cpp but trimmed.
class InlineGateway : public rclcpp::Node
{
public:
    InlineGateway() : rclcpp::Node("inline_gateway")
    {
        auto cfg = sg::load_config_string(CONFIG);
        for (const auto & spec : cfg.sensors) {
            auto pipe  = std::make_shared<Pipe>();
            pipe->spec = spec;
            pipe->view = std::make_unique<sg::MessageView>(spec.type);
            pipe->pub  = create_publisher<sg::msg::SmartData>(
                spec.output_topic, rclcpp::QoS(200));
            Pipe * p = pipe.get();
            pipe->sub  = create_generic_subscription(
                spec.topic, spec.type, rclcpp::SensorDataQoS(),
                [this, p](std::shared_ptr<rclcpp::SerializedMessage> m) {
                    const auto t0 = std::chrono::steady_clock::now();
                    p->view->deserialize(*m);
                    auto v = sg::decompose(p->spec, *p->view, 1, 2, 0);
                    for (const auto & sd : v) p->pub->publish(sd);
                    const auto t1 = std::chrono::steady_clock::now();
                    latencies_us.push_back(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count() / 1000.0);
                    ++p->msgs;
                    p->sd_count += v.size();
                });
            pipes_.push_back(pipe);
        }
    }

    std::size_t laser_sd() const { return pipes_[0]->sd_count; }
    std::size_t imu_sd()   const { return pipes_[1]->sd_count; }
    std::size_t temp_sd()  const { return pipes_[2]->sd_count; }

    std::vector<double> latencies_us;

private:
    struct Pipe {
        sg::SensorSpec spec;
        std::unique_ptr<sg::MessageView> view;
        rclcpp::Publisher<sg::msg::SmartData>::SharedPtr pub;
        std::shared_ptr<rclcpp::GenericSubscription>     sub;
        std::size_t msgs{0}, sd_count{0};
    };
    std::vector<std::shared_ptr<Pipe>> pipes_;
};

class Source : public rclcpp::Node
{
public:
    Source() : rclcpp::Node("source")
    {
        laser_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan",        rclcpp::SensorDataQoS());
        imu_pub_   = create_publisher<sensor_msgs::msg::Imu>      ("/imu/data",    rclcpp::SensorDataQoS());
        temp_pub_  = create_publisher<sensor_msgs::msg::Temperature>("/temperature", 10);
        laser_timer_ = create_wall_timer(100ms, [this] { publish_scan(); });
        imu_timer_   = create_wall_timer( 20ms, [this] { publish_imu(); });
        temp_timer_  = create_wall_timer(200ms, [this] { publish_temp(); });
    }
    std::atomic<std::size_t> scans{0}, imus{0}, temps{0};

private:
    void publish_scan()
    {
        sensor_msgs::msg::LaserScan s;
        s.header.stamp = now(); s.header.frame_id = "laser_frame";
        s.angle_min       = -static_cast<float>(M_PI);
        s.angle_max       =  static_cast<float>(M_PI);
        s.angle_increment = 2.0f * static_cast<float>(M_PI) / LASER_BEAMS;
        s.range_min = 0.1f; s.range_max = 30.0f;
        s.ranges.assign(LASER_BEAMS, 5.0f);
        laser_pub_->publish(s); ++scans;
    }
    void publish_imu()
    {
        sensor_msgs::msg::Imu m;
        m.header.stamp = now(); m.header.frame_id = "imu_frame";
        m.linear_acceleration.z = 9.81;
        imu_pub_->publish(m); ++imus;
    }
    void publish_temp()
    {
        sensor_msgs::msg::Temperature t;
        t.header.stamp = now(); t.header.frame_id = "base_link";
        t.temperature = 25.0;
        temp_pub_->publish(t); ++temps;
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr   laser_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr         imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::TimerBase::SharedPtr laser_timer_, imu_timer_, temp_timer_;
};

class Sink : public rclcpp::Node
{
public:
    Sink() : rclcpp::Node("sink")
    {
        auto qos = rclcpp::QoS(200);
        sub_laser_ = create_subscription<sg::msg::SmartData>(
            "/smartdata/scan", qos,
            [this](sg::msg::SmartData::ConstSharedPtr m) {
                if (m->unit == si_units::METER) ++laser;
            });
        sub_imu_ = create_subscription<sg::msg::SmartData>(
            "/smartdata/imu/data", qos,
            [this](sg::msg::SmartData::ConstSharedPtr m) {
                if (m->unit == si_units::METER_PER_SEC_SQ) ++imu_accel;
                else if (m->unit == si_units::RADIAN_PER_SECOND) ++imu_gyro;
            });
        sub_temp_ = create_subscription<sg::msg::SmartData>(
            "/smartdata/temperature", 10,
            [this](sg::msg::SmartData::ConstSharedPtr m) {
                if (m->unit == si_units::KELVIN) ++temp;
            });
    }
    std::atomic<std::size_t> laser{0}, imu_accel{0}, imu_gyro{0}, temp{0};
private:
    rclcpp::Subscription<sg::msg::SmartData>::SharedPtr sub_laser_, sub_imu_, sub_temp_;
};

}  // namespace

class GatewayPipelineTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        if (!rclcpp::ok()) rclcpp::init(0, nullptr);
        gw_     = std::make_shared<InlineGateway>();
        source_ = std::make_shared<Source>();
        sink_   = std::make_shared<Sink>();
        exec_   = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        exec_->add_node(gw_);
        exec_->add_node(source_);
        exec_->add_node(sink_);
    }
    void TearDown() override { exec_->cancel(); }

    void run_for(std::chrono::milliseconds d)
    {
        const auto deadline = std::chrono::steady_clock::now() + d;
        while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
            exec_->spin_some(std::chrono::milliseconds(10));
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    std::shared_ptr<InlineGateway> gw_;
    std::shared_ptr<Source>        source_;
    std::shared_ptr<Sink>          sink_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
};

TEST_F(GatewayPipelineTest, FanOutMatchesConfig)
{
    run_for(1500ms);
    const std::size_t scans = source_->scans;
    const std::size_t imus  = source_->imus;
    const std::size_t temps = source_->temps;
    ASSERT_GT(scans, 5u);
    ASSERT_GT(imus,  20u);

    EXPECT_GE(sink_->laser.load(),     (scans * LASER_BEAMS * 7) / 10);
    EXPECT_GE(sink_->imu_accel.load(), (imus  * 3 * 7) / 10);
    EXPECT_GE(sink_->imu_gyro.load(),  (imus  * 3 * 7) / 10);
    EXPECT_GE(sink_->temp.load(),      (temps * 7) / 10);
    GTEST_LOG_(INFO) << "scans=" << scans << " laser_sd=" << sink_->laser
                     << "\nimus=" << imus  << " accel="    << sink_->imu_accel
                     << " gyro=" << sink_->imu_gyro
                     << "\ntemps=" << temps << " temp_sd=" << sink_->temp;
}

TEST_F(GatewayPipelineTest, EndToEndCallbackP95Under1ms)
{
    run_for(1500ms);
    auto & v = gw_->latencies_us;
    ASSERT_GT(v.size(), 20u);
    std::sort(v.begin(), v.end());
    const double p50 = v[v.size() / 2];
    const double p95 = v[(v.size() * 95) / 100];
    GTEST_LOG_(INFO) << "deserialize+decompose+publish  p50=" << p50
                     << "  p95=" << p95 << "  n=" << v.size();
    EXPECT_LT(p95, 1000.0);
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    const int rc = RUN_ALL_TESTS();
    if (rclcpp::ok()) rclcpp::shutdown();
    return rc;
}
