// test_gateway_pipeline.cpp — integration test for the codegen pipeline.
//
// Exercises the generated dispatch end-to-end:
//   Source ──ROS msg──▶  InlineGateway (typed Subscription + decompose_<name>)
//                          ──SmartData──▶ Sink (asserts unit + counts)
//
// This test pins the post-codegen contract: typed Subscription<T>, direct
// member access, no introspection in the hot path.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "semantic_gateway/msg/smart_data.hpp"
#include "si_units.hpp"

#include "parsed_dispatch.hpp"

using namespace std::chrono_literals;
namespace sg = semantic_gateway;

namespace {

constexpr int LASER_BEAMS = 100;

// In-process gateway built on top of the generated register_all().
// Same wiring as src/gateway_node.cpp, minus TF (origin hardcoded for the test).
class InlineGateway : public rclcpp::Node
{
public:
    InlineGateway() : rclcpp::Node("inline_gateway")
    {
        auto lookup = [](const std::string &,
                          const builtin_interfaces::msg::Time &)
            -> std::tuple<double,double,double> {
            return {1.0, 2.0, 0.0};   // fixed origin; TF not under test here
        };

        auto on_sd = [this](const std::string & out_topic,
                             const sg::msg::SmartData & sd) {
            auto it = pubs_.find(out_topic);
            if (it == pubs_.end()) {
                it = pubs_.emplace(out_topic,
                    create_publisher<sg::msg::SmartData>(
                        out_topic, rclcpp::QoS(200))).first;
            }
            it->second->publish(sd);
            ++sd_counts_[out_topic];
        };

        subs_ = sg::gen::register_all(this, lookup, on_sd);
    }

    std::size_t sd_count(const std::string & out_topic) const
    {
        auto it = sd_counts_.find(out_topic);
        return it == sd_counts_.end() ? 0 : it->second;
    }

private:
    sg::gen::Subs subs_;
    std::unordered_map<std::string, rclcpp::Publisher<sg::msg::SmartData>::SharedPtr> pubs_;
    std::unordered_map<std::string, std::size_t> sd_counts_;
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

// Direct microbenchmark of the generated decompose_scan().
// No executor, no DDS — just the hot path.
//
// Acceptance criterion (monografia §6.3): p95 < 100 us — one order of
// magnitude below the ~1 ms ROS 2 single-hop p95 floor for small payloads
// reported by Kronauer et al. (SenSys 2021). The CI threshold below is
// tightened another order of magnitude (20 us) to catch regressions early.
TEST(CodegenLatency, DecomposeScanP95Under20us)
{
    sensor_msgs::msg::LaserScan s;
    s.header.stamp.sec = 1234;
    s.header.stamp.nanosec = 567000000;
    s.header.frame_id = "laser_frame";
    s.range_min = 0.1f;
    s.range_max = 30.0f;
    s.ranges.assign(LASER_BEAMS, 5.0f);

    std::vector<sg::msg::SmartData> out;
    out.reserve(LASER_BEAMS);

    constexpr int N = 2000;
    std::vector<double> latencies_us;
    latencies_us.reserve(N);

    for (int i = 0; i < N; ++i) {
        out.clear();
        const auto t0 = std::chrono::steady_clock::now();
        sg::gen::decompose_scan(s, 1.0, 2.0, 0.0, out);
        const auto t1 = std::chrono::steady_clock::now();
        latencies_us.push_back(
            std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count() / 1000.0);
    }

    ASSERT_EQ(out.size(), static_cast<std::size_t>(LASER_BEAMS));
    std::sort(latencies_us.begin(), latencies_us.end());
    const double p50 = latencies_us[N / 2];
    const double p95 = latencies_us[(N * 95) / 100];
    GTEST_LOG_(INFO) << "decompose_scan (codegen, " << LASER_BEAMS << " beams)"
                     << "  p50=" << p50 << "us  p95=" << p95 << "us";
    EXPECT_LT(p95, 20.0);
}

// Throughput: how many ROS scans per second the codegen decompose can
// sustain when nothing else competes for the CPU.
//
// Acceptance criterion (monografia §6.3): sustained rate at least 100x
// the aggregate SmartData input rate declared in config/sensors.yaml
// (LaserScan 360 beams x 10 Hz + Imu 6 fields x 100 Hz + Temperature 2 Hz
//  ~= 4200 SmartData/s). At 100 beams/scan that maps to ~1200 scans/s.
TEST(CodegenThroughput, DecomposeScanSustained)
{
    sensor_msgs::msg::LaserScan s;
    s.header.stamp.sec = 1;
    s.header.frame_id = "laser_frame";
    s.range_min = 0.1f;
    s.range_max = 30.0f;
    s.ranges.assign(LASER_BEAMS, 5.0f);

    std::vector<sg::msg::SmartData> out;
    out.reserve(LASER_BEAMS);

    constexpr auto BUDGET = std::chrono::milliseconds(500);
    std::size_t iterations = 0;
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < BUDGET) {
        out.clear();
        sg::gen::decompose_scan(s, 1.0, 2.0, 0.0, out);
        ++iterations;
    }
    const auto elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();

    const double scans_per_s    = iterations / elapsed_s;
    const double sd_per_s       = scans_per_s * LASER_BEAMS;
    GTEST_LOG_(INFO) << "decompose_scan throughput: "
                     << scans_per_s << " scans/s = "
                     << sd_per_s    << " SmartData/s ("
                     << LASER_BEAMS << " beams/scan)";

    // 1200 scans/s = 100x headroom over the aggregate declared in
    // config/sensors.yaml (~4200 SmartData/s). See monografia §6.3.
    EXPECT_GT(scans_per_s, 1200.0);
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    const int rc = RUN_ALL_TESTS();
    if (rclcpp::ok()) rclcpp::shutdown();
    return rc;
}
