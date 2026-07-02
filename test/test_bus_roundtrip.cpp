// test_bus_roundtrip.cpp — end-to-end fidelity test for the SmartData bus.
//
//   ROS publisher ──▶ Gateway ──▶ SDBus (multicast) ──▶ sd_to_ros consumer
//                                                          ──▶ ROS subscriber
//
// Asserts that what comes out of sd_to_ros matches what went in through
// the gateway, field-by-field, for each decomposition kind.
//
// To keep the test hermetic the gateway and sd_to_ros run in-process as
// classes (not the actual executables); they exercise the same codegen
// headers the real nodes use. A random multicast port is chosen per test
// to avoid colliding with other tests or live SDAV traffic.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "semantic_gateway/msg/smart_data.hpp"

#include "sd_bus.hpp"
#include "parsed_dispatch.hpp"
#include "parsed_sd_dispatch.hpp"

namespace sg = semantic_gateway;
using namespace std::chrono_literals;

namespace {

uint16_t random_port()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(45000, 49000);
    return static_cast<uint16_t>(dis(gen));
}

constexpr const char * MCAST_GROUP = "224.1.1.7";  // distinct from default

// ── Gateway under test ──────────────────────────────────────────────────────
class GatewayUT : public rclcpp::Node {
public:
    GatewayUT(const std::string & group, uint16_t port)
      : rclcpp::Node("gateway_ut"),
        bus_(std::make_unique<sg::SDBus>(group, port))
    {
        auto lookup = [](const std::string &,
                          const builtin_interfaces::msg::Time &)
            -> std::tuple<double,double,double> {
            return {1.0, 2.0, 0.0};
        };
        auto on_sd = [this](const std::string &, const sg::msg::SmartData & sd) {
            bus_->broadcast(sd);
        };
        subs_ = sg::gen::register_all(this, lookup, on_sd);
    }

private:
    std::unique_ptr<sg::SDBus> bus_;
    sg::gen::Subs subs_;
};

// ── sd_to_ros under test ────────────────────────────────────────────────────
class SdToRosUT : public rclcpp::Node {
public:
    SdToRosUT(const std::string & group, uint16_t port)
      : rclcpp::Node("sd_to_ros_ut"),
        bus_(std::make_unique<sg::SDBus>(group, port))
    {
        pub_scan_ = create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan_recovered", rclcpp::SensorDataQoS());
        pub_imu_  = create_publisher<sensor_msgs::msg::Imu>(
            "/imu/data_recovered", rclcpp::SensorDataQoS());
        pub_temp_ = create_publisher<sensor_msgs::msg::Temperature>(
            "/temperature_recovered", 10);

        consumers_.on_scan = [this](const sensor_msgs::msg::LaserScan & m) {
            pub_scan_->publish(m);
        };
        consumers_.on_imu_data = [this](const sensor_msgs::msg::Imu & m) {
            pub_imu_->publish(m);
        };
        consumers_.on_temperature = [this](const sensor_msgs::msg::Temperature & m) {
            pub_temp_->publish(m);
        };

        bus_->listen([this](const sg::msg::SmartData & sd) {
            std::lock_guard<std::mutex> lk(mu_);
            sg::gen::dispatch_sd(consumers_, sd);
        });

        drain_timer_ = create_wall_timer(20ms, [this] {
            std::lock_guard<std::mutex> lk(mu_);
            sg::gen::drain(consumers_);
        });
    }

    ~SdToRosUT() override { if (bus_) bus_->stop(); }

private:
    std::unique_ptr<sg::SDBus> bus_;
    sg::gen::Consumers consumers_;
    std::mutex mu_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr   pub_scan_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr         pub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp_;
    rclcpp::TimerBase::SharedPtr drain_timer_;
};

// ── Sources & sinks ─────────────────────────────────────────────────────────
class Sources : public rclcpp::Node {
public:
    Sources() : rclcpp::Node("sources")
    {
        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan",
            rclcpp::SensorDataQoS());
        imu_pub_  = create_publisher<sensor_msgs::msg::Imu>("/imu/data",
            rclcpp::SensorDataQoS());
        temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>(
            "/temperature", 10);
    }

    void emit_scan(uint64_t t_us, const std::vector<float> & ranges)
    {
        sensor_msgs::msg::LaserScan s;
        s.header.stamp.sec     = static_cast<int32_t>(t_us / 1'000'000ull);
        s.header.stamp.nanosec = static_cast<uint32_t>((t_us % 1'000'000ull) * 1000ull);
        s.header.frame_id = "laser_frame";
        s.range_min = 0.0f; s.range_max = 100.0f;
        s.angle_min = -3.14f; s.angle_max = 3.14f;
        s.angle_increment = 0.0628f;
        s.ranges = ranges;
        scan_pub_->publish(s);
    }
    void emit_imu(uint64_t t_us, double ax, double ay, double az,
                                  double wx, double wy, double wz)
    {
        sensor_msgs::msg::Imu m;
        m.header.stamp.sec     = static_cast<int32_t>(t_us / 1'000'000ull);
        m.header.stamp.nanosec = static_cast<uint32_t>((t_us % 1'000'000ull) * 1000ull);
        m.header.frame_id = "imu_frame";
        m.linear_acceleration.x = ax;
        m.linear_acceleration.y = ay;
        m.linear_acceleration.z = az;
        m.angular_velocity.x = wx;
        m.angular_velocity.y = wy;
        m.angular_velocity.z = wz;
        imu_pub_->publish(m);
    }
    void emit_temp(uint64_t t_us, double celsius)
    {
        sensor_msgs::msg::Temperature t;
        t.header.stamp.sec     = static_cast<int32_t>(t_us / 1'000'000ull);
        t.header.stamp.nanosec = static_cast<uint32_t>((t_us % 1'000'000ull) * 1000ull);
        t.header.frame_id = "base_link";
        t.temperature = celsius;
        temp_pub_->publish(t);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr   scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr         imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
};

class Sinks : public rclcpp::Node {
public:
    Sinks() : rclcpp::Node("sinks")
    {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_recovered", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
                std::lock_guard<std::mutex> lk(mu_);
                last_scan = *m; ++n_scan;
            });
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_recovered", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
                std::lock_guard<std::mutex> lk(mu_);
                last_imu = *m; ++n_imu;
            });
        temp_sub_ = create_subscription<sensor_msgs::msg::Temperature>(
            "/temperature_recovered", 10,
            [this](sensor_msgs::msg::Temperature::ConstSharedPtr m) {
                std::lock_guard<std::mutex> lk(mu_);
                last_temp = *m; ++n_temp;
            });
    }

    std::mutex mu_;
    sensor_msgs::msg::LaserScan   last_scan;
    sensor_msgs::msg::Imu         last_imu;
    sensor_msgs::msg::Temperature last_temp;
    std::atomic<int> n_scan{0}, n_imu{0}, n_temp{0};

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub_;
};

}  // namespace

class BusRoundtripTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        if (!rclcpp::ok()) rclcpp::init(0, nullptr);
        const uint16_t port = random_port();
        sd_to_ros_ = std::make_shared<SdToRosUT>(MCAST_GROUP, port);
        gateway_   = std::make_shared<GatewayUT>(MCAST_GROUP, port);
        sources_   = std::make_shared<Sources>();
        sinks_     = std::make_shared<Sinks>();

        exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        exec_->add_node(gateway_);
        exec_->add_node(sd_to_ros_);
        exec_->add_node(sources_);
        exec_->add_node(sinks_);
    }

    void TearDown() override { exec_->cancel(); }

    void spin_for(std::chrono::milliseconds d)
    {
        const auto deadline = std::chrono::steady_clock::now() + d;
        while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
            exec_->spin_some(std::chrono::milliseconds(5));
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    std::shared_ptr<GatewayUT> gateway_;
    std::shared_ptr<SdToRosUT> sd_to_ros_;
    std::shared_ptr<Sources>   sources_;
    std::shared_ptr<Sinks>     sinks_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
};

TEST_F(BusRoundtripTest, TemperatureScalar)
{
    spin_for(200ms);  // let discovery settle
    sources_->emit_temp(1'000'000'000ull, 25.0);
    spin_for(400ms);

    ASSERT_GT(sinks_->n_temp.load(), 0);
    std::lock_guard<std::mutex> lk(sinks_->mu_);
    EXPECT_NEAR(sinks_->last_temp.temperature, 25.0, 1e-6)
        << "Kelvin → Celsius round-trip must restore the original value";
    EXPECT_EQ(sinks_->last_temp.header.stamp.sec, 1000);
    EXPECT_EQ(sinks_->last_temp.header.stamp.nanosec, 0u);
}

TEST_F(BusRoundtripTest, ImuComposite)
{
    spin_for(200ms);
    sources_->emit_imu(2'000'000'000ull,
                       0.1, 0.2, 9.81,    // accel
                       0.01, 0.02, 0.03); // gyro
    spin_for(400ms);

    ASSERT_GT(sinks_->n_imu.load(), 0);
    std::lock_guard<std::mutex> lk(sinks_->mu_);
    EXPECT_NEAR(sinks_->last_imu.linear_acceleration.x, 0.1,  1e-9);
    EXPECT_NEAR(sinks_->last_imu.linear_acceleration.y, 0.2,  1e-9);
    EXPECT_NEAR(sinks_->last_imu.linear_acceleration.z, 9.81, 1e-9);
    EXPECT_NEAR(sinks_->last_imu.angular_velocity.x, 0.01, 1e-9);
    EXPECT_NEAR(sinks_->last_imu.angular_velocity.y, 0.02, 1e-9);
    EXPECT_NEAR(sinks_->last_imu.angular_velocity.z, 0.03, 1e-9);
}

TEST_F(BusRoundtripTest, LaserScanArray)
{
    spin_for(200ms);
    std::vector<float> ranges;
    ranges.reserve(50);
    for (int i = 0; i < 50; ++i) ranges.push_back(0.5f + 0.1f * i);
    sources_->emit_scan(3'000'000'000ull, ranges);
    // Push a second scan with a different stamp to force flush of the first.
    spin_for(150ms);
    std::vector<float> ranges2(50, 0.0f);
    sources_->emit_scan(3'500'000'000ull, ranges2);
    spin_for(400ms);

    ASSERT_GT(sinks_->n_scan.load(), 0);
    std::lock_guard<std::mutex> lk(sinks_->mu_);
    ASSERT_EQ(sinks_->last_scan.ranges.size(), 50u)
        << "all 50 beams should have been reassembled";
    // We can't easily tell which scan was last; just verify per-beam fidelity
    // by checking that beams are either the first or second pattern.
    bool first  = std::fabs(sinks_->last_scan.ranges[0] - 0.5f) < 1e-5f;
    bool second = std::fabs(sinks_->last_scan.ranges[0] - 0.0f) < 1e-5f;
    ASSERT_TRUE(first || second);
    if (first) {
        for (int i = 0; i < 50; ++i) {
            EXPECT_NEAR(sinks_->last_scan.ranges[i], 0.5f + 0.1f * i, 1e-4f)
                << "beam " << i << " mismatch";
        }
    } else {
        for (int i = 0; i < 50; ++i) {
            EXPECT_NEAR(sinks_->last_scan.ranges[i], 0.0f, 1e-4f);
        }
    }
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    const int rc = RUN_ALL_TESTS();
    if (rclcpp::ok()) rclcpp::shutdown();
    return rc;
}
