// test_smartdata_marshaling.cpp — unit tests for the generic gateway core.
//
// Coverage:
//   * SI Unit dimensional encoder / decoder
//   * YAML → GatewayConfig parsing
//   * field introspection by dotted path on real ROS messages
//   * decomposer for scalar / array / composite specs
//   * latency budget: end-to-end decompose < 150 µs p95

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "rclcpp/serialization.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include "config_loader.hpp"
#include "decomposer.hpp"
#include "field_access.hpp"
#include "si_units.hpp"

using namespace semantic_gateway;
using namespace std::chrono;

// SI unit encoding/decoding

TEST(SiUnits, MeterIsPureLengthExponent)
{
    auto d = si_units::decode(si_units::METER);
    EXPECT_EQ(d.m, 1);
    EXPECT_EQ(d.kg, 0);
    EXPECT_EQ(d.s, 0);
}

TEST(SiUnits, MeterPerSecSqEncodesAccel)
{
    auto d = si_units::decode(si_units::METER_PER_SEC_SQ);
    EXPECT_EQ(d.m, 1);
    EXPECT_EQ(d.s, -2);
}

TEST(SiUnits, RadianPerSecondEncodesAngularVel)
{
    auto d = si_units::decode(si_units::RADIAN_PER_SECOND);
    EXPECT_EQ(d.rad, 1);
    EXPECT_EQ(d.s, -1);
}

TEST(SiUnits, VoltMatchesPaperFig1)
{
    auto d = si_units::decode(si_units::VOLT);
    EXPECT_EQ(d.kg, 1);
    EXPECT_EQ(d.m,  2);
    EXPECT_EQ(d.s, -3);
    EXPECT_EQ(d.A, -1);
}

TEST(SiUnits, ToStringRendersDimensions)
{
    EXPECT_EQ(si_units::to_string(si_units::METER), "m");
    EXPECT_EQ(si_units::to_string(si_units::METER_PER_SEC_SQ), "m·s⁻²");
    EXPECT_EQ(si_units::to_string(si_units::KELVIN), "K");
}

// ConfigLoader

TEST(ConfigLoader, ScalarSpecRoundTrip)
{
    const char * yaml = R"(
sensors:
  - topic: /temperature
    type: sensor_msgs/msg/Temperature
    decomposition: scalar
    value_field: temperature
    value_transform: { kind: linear, a: 1.0, b: 273.15 }
    unit: kelvin
    expiry_ms: 500
)";
    auto cfg = load_config_string(yaml);
    ASSERT_EQ(cfg.sensors.size(), 1u);
    const auto & s = cfg.sensors[0];
    EXPECT_EQ(s.topic, "/temperature");
    EXPECT_EQ(s.decomposition, Decomposition::Scalar);
    EXPECT_EQ(s.unit, si_units::KELVIN);
    EXPECT_EQ(s.expiry_us, 500'000ULL);
    EXPECT_EQ(s.output_topic, "/smartdata/temperature");
    EXPECT_DOUBLE_EQ(s.value_transform.a, 1.0);
    EXPECT_DOUBLE_EQ(s.value_transform.b, 273.15);
}

TEST(ConfigLoader, ArraySpecWithFilter)
{
    const char * yaml = R"(
sensors:
  - topic: /scan
    type: sensor_msgs/msg/LaserScan
    decomposition: array
    array_field: ranges
    unit: meter
    filter: { kind: finite_within, min_field: range_min, max_field: range_max }
    expiry_ms: 100
)";
    auto cfg = load_config_string(yaml);
    ASSERT_EQ(cfg.sensors.size(), 1u);
    const auto & s = cfg.sensors[0];
    EXPECT_EQ(s.decomposition, Decomposition::Array);
    EXPECT_EQ(s.array_field, "ranges");
    EXPECT_EQ(s.filter.kind, FilterKind::FiniteWithin);
    EXPECT_EQ(s.filter.min_field, "range_min");
}

TEST(ConfigLoader, CompositeSpecWithTwoGroups)
{
    const char * yaml = R"(
sensors:
  - topic: /imu/data
    type: sensor_msgs/msg/Imu
    decomposition: composite
    groups:
      - unit: meter_per_sec_sq
        fields: [linear_acceleration.x, linear_acceleration.y, linear_acceleration.z]
      - unit: rad_per_sec
        fields: [angular_velocity.x, angular_velocity.y, angular_velocity.z]
    expiry_ms: 10
)";
    auto cfg = load_config_string(yaml);
    const auto & s = cfg.sensors[0];
    EXPECT_EQ(s.decomposition, Decomposition::Composite);
    ASSERT_EQ(s.groups.size(), 2u);
    EXPECT_EQ(s.groups[0].unit, si_units::METER_PER_SEC_SQ);
    EXPECT_EQ(s.groups[0].fields.size(), 3u);
    EXPECT_EQ(s.groups[1].unit, si_units::RADIAN_PER_SECOND);
}

TEST(ConfigLoader, RejectsUnknownUnit)
{
    const char * yaml = R"(
sensors:
  - topic: /x
    type: sensor_msgs/msg/Temperature
    decomposition: scalar
    value_field: temperature
    unit: parsecs_per_fortnight
)";
    EXPECT_THROW(load_config_string(yaml), std::runtime_error);
}

// MessageView introspection

namespace {

template<typename T>
rclcpp::SerializedMessage serialize_msg(const T & msg)
{
    rclcpp::Serialization<T> ser;
    rclcpp::SerializedMessage out;
    ser.serialize_message(&msg, &out);
    return out;
}

sensor_msgs::msg::LaserScan make_scan(std::size_t n = 360)
{
    sensor_msgs::msg::LaserScan s;
    s.header.frame_id = "laser_frame";
    s.header.stamp.sec     = 1700000000;
    s.header.stamp.nanosec = 123'000'000;
    s.angle_min       = -static_cast<float>(M_PI);
    s.angle_max       =  static_cast<float>(M_PI);
    s.angle_increment = 2.0f * static_cast<float>(M_PI) / n;
    s.range_min       = 0.1f;
    s.range_max       = 30.0f;
    s.ranges.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        s.ranges[i] = 2.0f + 0.5f * std::sin(static_cast<float>(i));
    }
    return s;
}

sensor_msgs::msg::Imu make_imu()
{
    sensor_msgs::msg::Imu m;
    m.header.frame_id = "imu_frame";
    m.header.stamp.sec = 100; m.header.stamp.nanosec = 500'000'000;
    m.linear_acceleration.x = 0.1; m.linear_acceleration.y = -0.2; m.linear_acceleration.z = 9.81;
    m.angular_velocity.x    = 0.01; m.angular_velocity.y   = -0.02; m.angular_velocity.z    = 0.03;
    return m;
}

sensor_msgs::msg::Temperature make_temp()
{
    sensor_msgs::msg::Temperature t;
    t.header.frame_id = "base_link";
    t.header.stamp.sec = 100; t.header.stamp.nanosec = 0;
    t.temperature = 25.0;
    return t;
}

}  // namespace

TEST(MessageView, LaserScanIntrospection)
{
    auto scan = make_scan(50);
    auto bytes = serialize_msg(scan);
    MessageView mv("sensor_msgs/msg/LaserScan");
    mv.deserialize(bytes);
    EXPECT_EQ(mv.read_string("header.frame_id"), "laser_frame");
    EXPECT_EQ(mv.read_time_us("header.stamp"),
              1'700'000'000ULL * 1'000'000ULL + 123'000ULL);
    EXPECT_EQ(mv.array_size("ranges"), 50u);
    EXPECT_NEAR(mv.read_array_double("ranges", 0),
                static_cast<double>(scan.ranges[0]), 1e-6);
    EXPECT_NEAR(mv.read_double("range_min"), 0.1, 1e-6);
    EXPECT_NEAR(mv.read_double("range_max"), 30.0, 1e-6);
}

TEST(MessageView, ImuIntrospection)
{
    auto imu = make_imu();
    auto bytes = serialize_msg(imu);
    MessageView mv("sensor_msgs/msg/Imu");
    mv.deserialize(bytes);
    EXPECT_EQ(mv.read_string("header.frame_id"), "imu_frame");
    EXPECT_DOUBLE_EQ(mv.read_double("linear_acceleration.x"),  0.1);
    EXPECT_DOUBLE_EQ(mv.read_double("linear_acceleration.z"),  9.81);
    EXPECT_DOUBLE_EQ(mv.read_double("angular_velocity.y"),    -0.02);
}

TEST(MessageView, TemperatureIntrospection)
{
    auto t = make_temp();
    auto bytes = serialize_msg(t);
    MessageView mv("sensor_msgs/msg/Temperature");
    mv.deserialize(bytes);
    EXPECT_DOUBLE_EQ(mv.read_double("temperature"), 25.0);
    EXPECT_EQ(mv.read_string("header.frame_id"), "base_link");
}

// decomposer

SensorSpec make_laser_spec()
{
    SensorSpec s;
    s.topic = "/scan";
    s.type  = "sensor_msgs/msg/LaserScan";
    s.decomposition = Decomposition::Array;
    s.array_field = "ranges";
    s.filter.kind = FilterKind::FiniteWithin;
    s.filter.min_field = "range_min";
    s.filter.max_field = "range_max";
    s.unit = si_units::METER;
    s.expiry_us = 100'000;
    return s;
}

SensorSpec make_imu_spec()
{
    SensorSpec s;
    s.topic = "/imu/data";
    s.type  = "sensor_msgs/msg/Imu";
    s.decomposition = Decomposition::Composite;
    CompositeGroup g1{si_units::METER_PER_SEC_SQ,
                       {"linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"}};
    CompositeGroup g2{si_units::RADIAN_PER_SECOND,
                       {"angular_velocity.x",    "angular_velocity.y",    "angular_velocity.z"}};
    s.groups = {g1, g2};
    s.expiry_us = 10'000;
    return s;
}

SensorSpec make_temp_spec()
{
    SensorSpec s;
    s.topic = "/temperature";
    s.type  = "sensor_msgs/msg/Temperature";
    s.decomposition = Decomposition::Scalar;
    s.value_field = "temperature";
    s.value_transform.kind = TransformKind::Linear;
    s.value_transform.a = 1.0;
    s.value_transform.b = 273.15;
    s.unit = si_units::KELVIN;
    s.expiry_us = 500'000;
    return s;
}

TEST(Decompose, LaserProducesOneSdPerBeam)
{
    auto scan = make_scan(50);
    auto bytes = serialize_msg(scan);
    MessageView mv("sensor_msgs/msg/LaserScan");
    mv.deserialize(bytes);
    auto out = decompose(make_laser_spec(), mv, 1.0, 2.0, 0.0);
    EXPECT_EQ(out.size(), 50u);
    for (uint32_t i = 0; i < out.size(); ++i) {
        EXPECT_EQ(out[i].unit, si_units::METER);
        EXPECT_EQ(out[i].origin_device, i);
        EXPECT_DOUBLE_EQ(out[i].origin_x, 1.0);
    }
}

TEST(Decompose, LaserFiltersOutOfRange)
{
    auto scan = make_scan(10);
    scan.ranges[0] = std::numeric_limits<float>::infinity();
    scan.ranges[1] = scan.range_max + 1.f;
    auto bytes = serialize_msg(scan);
    MessageView mv("sensor_msgs/msg/LaserScan");
    mv.deserialize(bytes);
    auto out = decompose(make_laser_spec(), mv, 0, 0, 0);
    EXPECT_EQ(out.size(), 8u);
    for (const auto & sd : out) EXPECT_GE(sd.origin_device, 2u);
}

TEST(Decompose, ImuProducesSixSd)
{
    auto imu = make_imu();
    auto bytes = serialize_msg(imu);
    MessageView mv("sensor_msgs/msg/Imu");
    mv.deserialize(bytes);
    auto out = decompose(make_imu_spec(), mv, 7, 8, 9);
    ASSERT_EQ(out.size(), 6u);
    for (int i = 0; i < 3; ++i) EXPECT_EQ(out[i].unit, si_units::METER_PER_SEC_SQ);
    for (int i = 3; i < 6; ++i) EXPECT_EQ(out[i].unit, si_units::RADIAN_PER_SECOND);
    EXPECT_DOUBLE_EQ(out[0].value, 0.1);
    EXPECT_DOUBLE_EQ(out[2].value, 9.81);
    EXPECT_DOUBLE_EQ(out[5].value, 0.03);
}

TEST(Decompose, TemperatureCelsiusToKelvin)
{
    auto t = make_temp();
    auto bytes = serialize_msg(t);
    MessageView mv("sensor_msgs/msg/Temperature");
    mv.deserialize(bytes);
    auto out = decompose(make_temp_spec(), mv, 0, 0, 0);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_DOUBLE_EQ(out[0].value, 298.15);
    EXPECT_EQ(out[0].unit, si_units::KELVIN);
}

// latency budget

TEST(Latency, LaserDecomposeP95Under150us)
{
    auto scan = make_scan(200);
    auto bytes = serialize_msg(scan);
    MessageView mv("sensor_msgs/msg/LaserScan");
    auto spec = make_laser_spec();

    std::vector<double> us;
    us.reserve(1000);
    for (int i = 0; i < 1000; ++i) {
        const auto t0 = steady_clock::now();
        mv.deserialize(bytes);
        auto out = decompose(spec, mv, 0, 0, 0);
        const auto t1 = steady_clock::now();
        ASSERT_FALSE(out.empty());
        us.push_back(duration_cast<nanoseconds>(t1 - t0).count() / 1000.0);
    }
    std::sort(us.begin(), us.end());
    const double p50 = us[us.size() / 2];
    const double p95 = us[(us.size() * 95) / 100];
    GTEST_LOG_(INFO) << "LaserScan(200 beams) deserialize+decompose  p50=" << p50
                     << " µs  p95=" << p95 << " µs";
    EXPECT_LT(p95, 150.0);
}

TEST(Latency, ImuDecomposeP95Under150us)
{
    auto imu = make_imu();
    auto bytes = serialize_msg(imu);
    MessageView mv("sensor_msgs/msg/Imu");
    auto spec = make_imu_spec();

    std::vector<double> us;
    us.reserve(1000);
    for (int i = 0; i < 1000; ++i) {
        const auto t0 = steady_clock::now();
        mv.deserialize(bytes);
        auto out = decompose(spec, mv, 0, 0, 0);
        const auto t1 = steady_clock::now();
        ASSERT_EQ(out.size(), 6u);
        us.push_back(duration_cast<nanoseconds>(t1 - t0).count() / 1000.0);
    }
    std::sort(us.begin(), us.end());
    const double p95 = us[(us.size() * 95) / 100];
    GTEST_LOG_(INFO) << "Imu deserialize+decompose  p95=" << p95 << " µs";
    EXPECT_LT(p95, 150.0);
}
