// test_gateway_pipeline.cpp — the generated transducers (parsed_transducers.hpp)
// and the cost of turning a ROS message into SmartData.
//
//   * each (topic x quantity) is one transducer class with its UNIT, TYPE
//     and a disjoint device block; instances read their own field;
//   * invalid vector elements keep their previous (expiring) value;
//   * unit conversions are applied on the way in and undone on the way out;
//   * actuator transducers write their field of the setpoint and publish it;
//   * latency / throughput of the translation (Source → transducers →
//     Responsive SmartData), with and without the network send.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "si_units.hpp"
#include "smartdata/network.hpp"
#include "smartdata/smartdata.hpp"
#include "parsed_transducers.hpp"

namespace gen = semantic_gateway::gen;
using namespace std::chrono_literals;

namespace {

sd::Origin origin_at(sd::Time t)
{
    sd::Origin o;
    o.x = 1.0;
    o.y = 2.0;
    o.t = t;
    o.confidence = 100;
    return o;
}

sensor_msgs::msg::LaserScan make_scan(std::size_t beams, float value)
{
    sensor_msgs::msg::LaserScan s;
    s.header.frame_id = "laser_frame";
    s.range_min = 0.1f;
    s.range_max = 30.0f;
    s.ranges.assign(beams, value);
    return s;
}

}  // namespace

TEST(Transducers, OneClassPerQuantityWithDisjointDevices)
{
    static_assert(gen::Scan_Transducer::UNIT == si_units::METER);
    static_assert(gen::ImuDataLinearAcceleration_Transducer::UNIT == si_units::METER_PER_SEC_SQ);
    static_assert(gen::ImuDataAngularVelocity_Transducer::UNIT == si_units::RADIAN_PER_SECOND);
    static_assert(gen::Temperature_Transducer::UNIT == si_units::KELVIN);
    static_assert(gen::CmdVelLinearVelocity_Transducer::UNIT == si_units::METER_PER_SECOND);

    static_assert(gen::Scan_Transducer::active);
    static_assert(gen::Scan_Transducer::TYPE == gen::Scan_Transducer::SENSOR);
    static_assert(gen::CmdVelLinearVelocity_Transducer::TYPE & gen::CmdVelLinearVelocity_Transducer::ACTUATOR);

    EXPECT_EQ(gen::ImuDataLinearAcceleration_Transducer::DEVICES, 3u);
    EXPECT_NE(gen::ImuDataLinearAcceleration_Transducer::DEV_BASE,
              gen::ImuLinearAcceleration_Transducer::DEV_BASE)
        << "same unit on two topics: different device blocks";
    EXPECT_EQ(gen::ImuDataAngularVelocity_Transducer::DEV_BASE -
              gen::ImuDataLinearAcceleration_Transducer::DEV_BASE, 0x10000u);
}

TEST(Transducers, CompositeInstancesReadTheirOwnField)
{
    sd::Source<sensor_msgs::msg::Imu> src;
    using Acc  = gen::ImuDataLinearAcceleration_Transducer;
    using Gyro = gen::ImuDataAngularVelocity_Transducer;
    Acc  ay(Acc::DEV_BASE + 1, src);
    Gyro wz(Gyro::DEV_BASE + 2, src);

    sensor_msgs::msg::Imu m;
    m.linear_acceleration.y = -0.5;
    m.angular_velocity.z    = 0.25;
    src.publish(m, origin_at(42));

    EXPECT_DOUBLE_EQ(ay.read().value, -0.5);
    EXPECT_DOUBLE_EQ(wz.read().value, 0.25);
    EXPECT_EQ(ay.read().origin.t, 42);
    EXPECT_DOUBLE_EQ(ay.read().origin.y, 2.0);
}

TEST(Transducers, InvalidBeamsKeepTheirExpiringValue)
{
    sd::Source<sensor_msgs::msg::LaserScan> src;
    using Beam = gen::Scan_Transducer;
    Beam b2(Beam::DEV_BASE + 2, src);

    auto s = make_scan(4, 1.0f);
    src.publish(s, origin_at(1));
    EXPECT_FLOAT_EQ(b2.read().value, 1.0f);

    s.ranges[2] = std::numeric_limits<float>::quiet_NaN();
    src.publish(s, origin_at(2));
    EXPECT_FLOAT_EQ(b2.read().value, 1.0f);
    EXPECT_EQ(b2.read().origin.t, 1) << "no update: the old sample simply expires";

    s.ranges[2] = 99.0f;  // beyond range_max
    src.publish(s, origin_at(3));
    EXPECT_EQ(b2.read().origin.t, 1);

    s.ranges.resize(2);   // shorter scan: this beam does not exist now
    src.publish(s, origin_at(4));
    EXPECT_EQ(b2.read().origin.t, 1);
}

TEST(Transducers, TemperatureIsKelvinOnTheWire)
{
    sd::Source<sensor_msgs::msg::Temperature> src;
    using T = gen::Temperature_Transducer;
    T t(T::DEV_BASE, src);
    sensor_msgs::msg::Temperature m;
    m.temperature = 25.0;
    src.publish(m, origin_at(1));
    EXPECT_NEAR(t.read().value, 298.15, 1e-9);
    EXPECT_NEAR(T::to_ros(t.read().value), 25.0, 1e-9);
}

TEST(Transducers, ActuatorWritesItsFieldOfTheSetpoint)
{
    std::vector<geometry_msgs::msg::Twist> published;
    sd::Sink<geometry_msgs::msg::Twist> sink(
        [&](const geometry_msgs::msg::Twist & m) { published.push_back(m); },
        [] { return origin_at(7); });

    using Lin = gen::CmdVelLinearVelocity_Transducer;
    using Ang = gen::CmdVelAngularVelocity_Transducer;
    Lin vx(Lin::DEV_BASE + 0, sink);
    Ang wz(Ang::DEV_BASE + 2, sink);

    vx.actuate(0.3);
    wz.actuate(-1.0);
    ASSERT_EQ(published.size(), 2u);
    EXPECT_DOUBLE_EQ(published.back().linear.x, 0.3) << "setpoint keeps the other fields";
    EXPECT_DOUBLE_EQ(published.back().angular.z, -1.0);
    EXPECT_DOUBLE_EQ(vx.read().value, 0.3) << "actuators report their state";
    EXPECT_EQ(vx.read().origin.t, 7);
}

// ── performance ──────────────────────────────────────────────────────────────
//
// Acceptance criterion (monografia, Plano de Experimentos): translating a
// message must stay below 100 µs at p95, an order of magnitude under the
// ~1 ms p95 floor of a ROS 2 hop for small payloads (Kronauer et al. 2021).
// "Translation" = the ROS message reaches the transducers, every instance
// updates and its Responsive SmartData is notified. The variant with an
// interested proxy also builds and sends one 78-byte Response per beam.

namespace {

constexpr std::size_t BEAMS = 100;

struct Scan_Rig {
    explicit Scan_Rig(bool interested)
    {
        sd::Network::Config c;
        c.group = "224.1.1.10";
        c.port  = static_cast<uint16_t>(std::uniform_int_distribution<int>(45000, 49000)(rng));
        c.interest_refresh = 100ms;
        net = std::make_unique<sd::Network>(c);
        for (std::size_t i = 0; i < BEAMS; ++i) {
            beams.push_back(std::make_unique<sd::Responsive_SmartData<gen::Scan_Transducer>>(
                *net, gen::Scan_Transducer::DEV_BASE + static_cast<sd::Device_Id>(i),
                100'000, sd::ADVERTISED, 0, src));
        }
        if (interested) {
            proxy = std::make_unique<sd::Interested_SmartData<si_units::METER>>(
                *net, sd::Region::everywhere(), 100'000, 0, sd::ALL);
            const auto deadline = std::chrono::steady_clock::now() + 2s;
            while (std::chrono::steady_clock::now() < deadline &&
                   beams.back()->interested() == 0) {
                std::this_thread::sleep_for(5ms);
            }
        }
    }

    std::mt19937 rng{std::random_device{}()};
    sd::Source<sensor_msgs::msg::LaserScan> src;
    std::unique_ptr<sd::Network> net;
    std::vector<std::unique_ptr<sd::Responsive_SmartData<gen::Scan_Transducer>>> beams;
    std::unique_ptr<sd::Interested_SmartData<si_units::METER>> proxy;
};

struct Percentiles { double p50, p95; };

Percentiles measure(Scan_Rig & rig, int n)
{
    auto scan = make_scan(BEAMS, 5.0f);
    std::vector<double> us;
    us.reserve(n);
    for (int i = 0; i < n; ++i) {
        const auto o = origin_at(1'000'000 + i);
        const auto t0 = std::chrono::steady_clock::now();
        rig.src.publish(scan, o);
        const auto t1 = std::chrono::steady_clock::now();
        us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
    std::sort(us.begin(), us.end());
    return {us[n / 2], us[(n * 95) / 100]};
}

}  // namespace

TEST(Performance, TranslationLatencyP95Under100us)
{
    Scan_Rig rig(false);
    const auto p = measure(rig, 20'000);
    GTEST_LOG_(INFO) << "translate LaserScan (" << BEAMS << " beams): p50=" << p.p50
                     << "us p95=" << p.p95 << "us";
    EXPECT_LT(p.p95, 100.0);
}

TEST(Performance, TranslationPlusSendLatency)
{
    Scan_Rig rig(true);
    ASSERT_GT(rig.beams.back()->interested(), 0u) << "the proxy must have bound";
    const auto p = measure(rig, 2'000);
    GTEST_LOG_(INFO) << "translate + send LaserScan (" << BEAMS << " responses): p50="
                     << p.p50 << "us p95=" << p.p95 << "us";
    EXPECT_LT(p.p95, 20'000.0);  // sanity bound; the value is reported
}

TEST(Performance, SustainedThroughput)
{
    // 100x the aggregate declared in config/sensors.yaml (~4200 SmartData/s:
    // 360 beams x 10 Hz + 6 IMU fields x 100 Hz + temperature) = 4.2e5
    // updates/s → 4200 scans/s of BEAMS (100) beams
    Scan_Rig rig(false);
    auto scan = make_scan(BEAMS, 5.0f);
    std::size_t it = 0;
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < 500ms) {
        rig.src.publish(scan, origin_at(static_cast<sd::Time>(++it)));
    }
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const double scans = it / secs;
    GTEST_LOG_(INFO) << "translation throughput: " << scans << " scans/s = "
                     << scans * BEAMS << " SmartData updates/s";
    EXPECT_GT(scans * BEAMS, 4.2e5);

    Scan_Rig sending(true);
    std::size_t sent_it = 0;
    const auto s0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - s0 < 500ms) {
        sending.src.publish(scan, origin_at(static_cast<sd::Time>(++sent_it)));
    }
    const double ssecs = std::chrono::duration<double>(std::chrono::steady_clock::now() - s0).count();
    GTEST_LOG_(INFO) << "with network send: " << sent_it / ssecs << " scans/s = "
                     << sending.net->sent() / ssecs << " datagrams/s";
    EXPECT_GT(sent_it / ssecs * BEAMS, 4200.0) << "must at least keep up with the declared sensors";
}
