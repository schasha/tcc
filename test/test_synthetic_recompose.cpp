// test_synthetic_recompose.cpp — reconstruct ROS messages from SmartData
// that we *build by hand*, never going through decompose_<name>.
//
// Why this is different from test_validator.cpp:
//   - test_validator starts from a ROS message and tests the round-trip
//     decompose ↔ feed. It cannot catch a dispatcher that misroutes by
//     unit/dev_base, because the SmartData it tests are well-formed by
//     construction (the decomposer made them).
//   - This test simulates an external SmartData producer (e.g. a real
//     SDAV transducer on the wire): we fabricate SmartData envelopes
//     with chosen unit, origin_device and value, then feed them through
//     dispatch_sd(Consumers&, sd) — the exact entry point the
//     sd_to_ros_node uses on the bus side.
//
// Pipeline tested (no nodes, no UDP, no DDS):
//
//   make_sd(...)  ──dispatch_sd──▶  Consumers::on_<topic>(reconstructed)
//
// Coverage:
//   1) scalar     — synthetic Temperature in Kelvin → °C reconstruction
//   2) composite  — 6 hand-built IMU SDs, completion-triggered emission
//   3) array      — 16 hand-built LaserScan beams + origin_time transition flush
//   4) routing    — interleaved SDs from different sensors at once
//   5) rejection  — SDs with wrong unit or out-of-range device are dropped

#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "semantic_gateway/msg/smart_data.hpp"
#include "sd_codec.hpp"
#include "si_units.hpp"

#include "parsed_recompose.hpp"
#include "parsed_sd_dispatch.hpp"

namespace sg = semantic_gateway;

namespace {

// dev_base allocation must mirror codegen/generate.py's i * 0x10000 order,
// which follows the sensor order in config/sensors.yaml.
constexpr uint32_t DEV_BASE_SCAN        = 0x00000u;  // sensor 0
constexpr uint32_t DEV_BASE_IMU_DATA    = 0x10000u;  // sensor 1
constexpr uint32_t DEV_BASE_IMU         = 0x20000u;  // sensor 2 (unused here)
constexpr uint32_t DEV_BASE_TEMPERATURE = 0x30000u;  // sensor 3

constexpr uint64_t T0      = 1'700'000'000'000'000ull;  // µs
constexpr uint64_t EXPIRY  = 100'000ull;

}  // namespace

TEST(SyntheticRecomposeTest, Scalar_TemperatureFromKelvinSmartData)
{
    // Build the SmartData a remote producer would emit for a 23.5 °C reading:
    // unit = KELVIN, value already in K (the gateway protocol always carries SI).
    const auto sd = sg::make_sd(
        si_units::KELVIN, /*value=*/296.65,
        /*ox=*/1.0, /*oy=*/2.0, /*oz=*/0.0,
        /*dev=*/DEV_BASE_TEMPERATURE, /*t_us=*/T0, /*expiry=*/EXPIRY);

    sg::gen::Consumers c;
    std::optional<sensor_msgs::msg::Temperature> got;
    c.on_temperature = [&got](const sensor_msgs::msg::Temperature & m) {
        got = m;
    };

    sg::gen::dispatch_sd(c, sd);

    ASSERT_TRUE(got.has_value())
        << "scalar must emit immediately on first SD";
    EXPECT_NEAR(got->temperature, 23.5, 1e-9)
        << "recompose must invert the +273.15 K offset";
    // origin_time (µs) must land back as sec + nanosec
    EXPECT_EQ(got->header.stamp.sec, static_cast<int32_t>(T0 / 1'000'000ull));
    EXPECT_EQ(got->header.stamp.nanosec,
              static_cast<uint32_t>((T0 % 1'000'000ull) * 1000ull));
}

TEST(SyntheticRecomposeTest, Composite_ImuFromSixHandBuiltSmartData)
{
    // 6 SDs for /imu/data — 3 accel + 3 gyro, in the order codegen allocates:
    //   dev_base+0..2: linear_acceleration.{x,y,z}  (m/s²)
    //   dev_base+3..5: angular_velocity.{x,y,z}     (rad/s)
    const double ax = 0.10, ay = -0.20, az = 9.81;
    const double gx = 0.01, gy = -0.02, gz =  0.03;

    std::vector<sg::msg::SmartData> stream = {
        sg::make_sd(si_units::METER_PER_SEC_SQ,  ax, 0, 0, 0,
                    DEV_BASE_IMU_DATA + 0, T0, EXPIRY),
        sg::make_sd(si_units::METER_PER_SEC_SQ,  ay, 0, 0, 0,
                    DEV_BASE_IMU_DATA + 1, T0, EXPIRY),
        sg::make_sd(si_units::METER_PER_SEC_SQ,  az, 0, 0, 0,
                    DEV_BASE_IMU_DATA + 2, T0, EXPIRY),
        sg::make_sd(si_units::RADIAN_PER_SECOND, gx, 0, 0, 0,
                    DEV_BASE_IMU_DATA + 3, T0, EXPIRY),
        sg::make_sd(si_units::RADIAN_PER_SECOND, gy, 0, 0, 0,
                    DEV_BASE_IMU_DATA + 4, T0, EXPIRY),
        sg::make_sd(si_units::RADIAN_PER_SECOND, gz, 0, 0, 0,
                    DEV_BASE_IMU_DATA + 5, T0, EXPIRY),
    };

    sg::gen::Consumers c;
    int emitted = 0;
    std::optional<sensor_msgs::msg::Imu> got;
    c.on_imu_data = [&](const sensor_msgs::msg::Imu & m) {
        got = m;
        ++emitted;
    };

    for (const auto & sd : stream) sg::gen::dispatch_sd(c, sd);

    EXPECT_EQ(emitted, 1) << "composite emits exactly once when EXPECTED=6";
    ASSERT_TRUE(got.has_value());
    EXPECT_DOUBLE_EQ(got->linear_acceleration.x, ax);
    EXPECT_DOUBLE_EQ(got->linear_acceleration.y, ay);
    EXPECT_DOUBLE_EQ(got->linear_acceleration.z, az);
    EXPECT_DOUBLE_EQ(got->angular_velocity.x,    gx);
    EXPECT_DOUBLE_EQ(got->angular_velocity.y,    gy);
    EXPECT_DOUBLE_EQ(got->angular_velocity.z,    gz);
}

TEST(SyntheticRecomposeTest, Array_LaserScanFlushOnOriginTimeTransition)
{
    // Two synthetic scans (16 beams each) at different origin_times. The
    // recompose path keys batches by origin_time, so the first scan must
    // flush when the second one starts arriving.
    constexpr uint32_t N = 16;

    auto make_scan = [](uint64_t t, double base) {
        std::vector<sg::msg::SmartData> v;
        v.reserve(N);
        for (uint32_t i = 0; i < N; ++i) {
            v.push_back(sg::make_sd(
                si_units::METER, base + 0.1 * i,
                0, 0, 0,
                DEV_BASE_SCAN + i, t, EXPIRY));
        }
        return v;
    };

    const uint64_t t_a = T0;
    const uint64_t t_b = T0 + 100'000;  // 100 ms later
    const auto scan_a = make_scan(t_a, /*base=*/1.0);
    const auto scan_b = make_scan(t_b, /*base=*/5.0);

    sg::gen::Consumers c;
    std::vector<sensor_msgs::msg::LaserScan> emitted;
    c.on_scan = [&emitted](const sensor_msgs::msg::LaserScan & m) {
        emitted.push_back(m);
    };

    for (const auto & sd : scan_a) sg::gen::dispatch_sd(c, sd);
    EXPECT_EQ(emitted.size(), 0u)
        << "no emission yet — array doesn't know the burst is complete";

    // Feeding the first beam of scan B should flush scan A.
    sg::gen::dispatch_sd(c, scan_b[0]);
    ASSERT_EQ(emitted.size(), 1u)
        << "scan A must flush when origin_time transitions";

    // Drain the second scan explicitly (no further transition will come).
    for (std::size_t i = 1; i < scan_b.size(); ++i) {
        sg::gen::dispatch_sd(c, scan_b[i]);
    }
    sg::gen::drain(c);

    ASSERT_EQ(emitted.size(), 2u) << "drain must flush the pending second scan";

    // Verify scan A's values reconstruct exactly.
    const auto & a = emitted[0];
    ASSERT_EQ(a.ranges.size(), N);
    for (uint32_t i = 0; i < N; ++i) {
        EXPECT_NEAR(a.ranges[i], 1.0f + 0.1f * i, 1e-5f)
            << "scan A beam " << i;
    }
    // And scan B's.
    const auto & b = emitted[1];
    ASSERT_EQ(b.ranges.size(), N);
    for (uint32_t i = 0; i < N; ++i) {
        EXPECT_NEAR(b.ranges[i], 5.0f + 0.1f * i, 1e-5f)
            << "scan B beam " << i;
    }
}

TEST(SyntheticRecomposeTest, Routing_InterleavedStreamFromMultipleSensors)
{
    // Imitates the bus: SDs from /scan, /imu/data and /temperature arrive
    // mixed in arrival order. dispatch_sd must route each one correctly
    // by (unit, dev_base range) and only the right consumer must fire.
    sg::gen::Consumers c;
    std::optional<sensor_msgs::msg::Temperature> got_temp;
    std::optional<sensor_msgs::msg::Imu>         got_imu;
    int scan_emits = 0;
    c.on_temperature = [&got_temp](const sensor_msgs::msg::Temperature & m) {
        got_temp = m;
    };
    c.on_imu_data = [&got_imu](const sensor_msgs::msg::Imu & m) {
        got_imu = m;
    };
    c.on_scan = [&scan_emits](const sensor_msgs::msg::LaserScan &) {
        ++scan_emits;
    };

    // Interleave: imu accel x, temp, imu accel y, scan beam 0, imu accel z,
    // imu gyro x, scan beam 1, imu gyro y, imu gyro z (completes IMU).
    std::vector<sg::msg::SmartData> stream = {
        sg::make_sd(si_units::METER_PER_SEC_SQ, 0.1, 0, 0, 0,
                    DEV_BASE_IMU_DATA + 0, T0, EXPIRY),
        sg::make_sd(si_units::KELVIN, 300.0, 0, 0, 0,
                    DEV_BASE_TEMPERATURE, T0, EXPIRY),
        sg::make_sd(si_units::METER_PER_SEC_SQ, 0.2, 0, 0, 0,
                    DEV_BASE_IMU_DATA + 1, T0, EXPIRY),
        sg::make_sd(si_units::METER, 1.0, 0, 0, 0,
                    DEV_BASE_SCAN + 0, T0, EXPIRY),
        sg::make_sd(si_units::METER_PER_SEC_SQ, 0.3, 0, 0, 0,
                    DEV_BASE_IMU_DATA + 2, T0, EXPIRY),
        sg::make_sd(si_units::RADIAN_PER_SECOND, 0.01, 0, 0, 0,
                    DEV_BASE_IMU_DATA + 3, T0, EXPIRY),
        sg::make_sd(si_units::METER, 1.1, 0, 0, 0,
                    DEV_BASE_SCAN + 1, T0, EXPIRY),
        sg::make_sd(si_units::RADIAN_PER_SECOND, 0.02, 0, 0, 0,
                    DEV_BASE_IMU_DATA + 4, T0, EXPIRY),
        sg::make_sd(si_units::RADIAN_PER_SECOND, 0.03, 0, 0, 0,
                    DEV_BASE_IMU_DATA + 5, T0, EXPIRY),
    };

    for (const auto & sd : stream) sg::gen::dispatch_sd(c, sd);

    // Temperature fired immediately (scalar).
    ASSERT_TRUE(got_temp.has_value());
    EXPECT_NEAR(got_temp->temperature, 300.0 - 273.15, 1e-9);

    // IMU completed when its 6th SD arrived.
    ASSERT_TRUE(got_imu.has_value());
    EXPECT_DOUBLE_EQ(got_imu->linear_acceleration.x, 0.1);
    EXPECT_DOUBLE_EQ(got_imu->linear_acceleration.z, 0.3);
    EXPECT_DOUBLE_EQ(got_imu->angular_velocity.z,    0.03);

    // Scan still pending (no origin_time transition, no drain yet).
    EXPECT_EQ(scan_emits, 0);

    sg::gen::drain(c);
    EXPECT_EQ(scan_emits, 1) << "drain must flush the pending scan";
}

TEST(SyntheticRecomposeTest, Rejection_WrongUnitAndOutOfRangeDevicesAreDropped)
{
    sg::gen::Consumers c;
    int temp_emits = 0, imu_emits = 0, scan_emits = 0;
    c.on_temperature = [&](const sensor_msgs::msg::Temperature &) { ++temp_emits; };
    c.on_imu_data    = [&](const sensor_msgs::msg::Imu &)         { ++imu_emits; };
    c.on_scan        = [&](const sensor_msgs::msg::LaserScan &)   { ++scan_emits; };

    // (a) Wrong unit on /temperature's dev_base — the scalar dispatcher's
    //     unit guard must reject it.
    sg::gen::dispatch_sd(c, sg::make_sd(
        si_units::METER, 999.0, 0, 0, 0,
        DEV_BASE_TEMPERATURE, T0, EXPIRY));

    // (b) Wrong unit on /scan's dev_base — same idea for an array sensor.
    sg::gen::dispatch_sd(c, sg::make_sd(
        si_units::KELVIN, 1.0, 0, 0, 0,
        DEV_BASE_SCAN + 0, T0, EXPIRY));

    // (c) Device way out of every allocated range. dev_base allocation is
    //     i * 0x10000 with 4 sensors → ranges end at 0x3FFFF. 0xFF0000 is
    //     well past that.
    sg::gen::dispatch_sd(c, sg::make_sd(
        si_units::METER, 1.0, 0, 0, 0,
        /*dev=*/0xFF0000u, T0, EXPIRY));

    sg::gen::drain(c);

    EXPECT_EQ(temp_emits, 0) << "wrong unit must not reach temperature consumer";
    EXPECT_EQ(scan_emits, 0) << "wrong unit / OOR device must not reach scan consumer";
    EXPECT_EQ(imu_emits,  0) << "no IMU input was provided";
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
