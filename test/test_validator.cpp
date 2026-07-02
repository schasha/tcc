// test_validator.cpp — reconstruction-based fidelity test for the codegen pair.
//
// Proposta §3.3: the validator's job is to reassemble the original ROS
// message from the SmartData burst the gateway produced for it, and check
// that every reconstructed field matches the source. This is the strongest
// definition of "no loss across the gateway".
//
// Pipeline tested (no nodes, no DDS, no bus — straight C++ codegen):
//
//   ROS msg  ──decompose_<name>──▶  SmartData[]  ──feed_<name>──▶  ROS msg'
//
// One TEST per decomposition kind (scalar, array+filter, composite) keeps
// the failure mode obvious. test_bus_roundtrip covers the additional
// transport stage (UDP multicast); this test isolates the codegen pair.

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "semantic_gateway/msg/smart_data.hpp"

#include "parsed_decompose.hpp"
#include "parsed_recompose.hpp"

namespace sg = semantic_gateway;

namespace {

constexpr int LASER_BEAMS = 32;

// Replays a SmartData burst through feed_<name> and returns the reassembled
// message, calling flush_<name> as a safety net for batches that need it.
template <typename State, typename RosMsg, typename Feed, typename Flush>
RosMsg replay(const std::vector<sg::msg::SmartData> & sds,
              State & state, Feed feed, Flush flush)
{
    std::optional<RosMsg> out;
    for (const auto & sd : sds) feed(state, sd, out);
    if (!out) flush(state, out);
    EXPECT_TRUE(out.has_value()) << "recompose produced no message";
    return out.value_or(RosMsg{});
}

}  // namespace

TEST(ValidatorTest, ScalarRoundTrip_Temperature)
{
    sensor_msgs::msg::Temperature src;
    src.header.stamp.sec = 1234;
    src.header.stamp.nanosec = 567'000'000;
    src.header.frame_id = "base_link";
    src.temperature = 23.5;  // °C

    std::vector<sg::msg::SmartData> sds;
    sg::gen::decompose_temperature(src, 1.0, 2.0, 0.0, sds);
    ASSERT_EQ(sds.size(), 1u) << "scalar → 1 SmartData";

    // Kelvin in the wire: 23.5 + 273.15 = 296.65
    EXPECT_NEAR(sds[0].value, 296.65, 1e-9);
    EXPECT_EQ(sds[0].unit, si_units::KELVIN);
    EXPECT_EQ(sds[0].origin_x, 1.0);
    EXPECT_EQ(sds[0].origin_y, 2.0);

    sg::gen::State_temperature st;
    auto recovered = replay<sg::gen::State_temperature, sensor_msgs::msg::Temperature>(
        sds, st, sg::gen::feed_temperature, sg::gen::flush_temperature);

    EXPECT_NEAR(recovered.temperature, src.temperature, 1e-9)
        << "K → °C round-trip must invert the +273.15 offset";
    EXPECT_EQ(recovered.header.stamp.sec, src.header.stamp.sec);
    EXPECT_EQ(recovered.header.stamp.nanosec, src.header.stamp.nanosec);
}

TEST(ValidatorTest, CompositeRoundTrip_Imu)
{
    sensor_msgs::msg::Imu src;
    src.header.stamp.sec = 42;
    src.header.stamp.nanosec = 500'000'000;
    src.header.frame_id = "imu_frame";
    src.linear_acceleration.x =  0.1;
    src.linear_acceleration.y = -0.2;
    src.linear_acceleration.z =  9.81;
    src.angular_velocity.x    =  0.01;
    src.angular_velocity.y    = -0.02;
    src.angular_velocity.z    =  0.03;

    std::vector<sg::msg::SmartData> sds;
    sg::gen::decompose_imu_data(src, 0.0, 0.0, 0.0, sds);
    ASSERT_EQ(sds.size(), 6u) << "3 accel + 3 gyro";

    // unit per group
    for (int i = 0; i < 3; ++i)
        EXPECT_EQ(sds[i].unit, si_units::METER_PER_SEC_SQ);
    for (int i = 3; i < 6; ++i)
        EXPECT_EQ(sds[i].unit, si_units::RADIAN_PER_SECOND);

    sg::gen::State_imu_data st;
    auto recovered = replay<sg::gen::State_imu_data, sensor_msgs::msg::Imu>(
        sds, st, sg::gen::feed_imu_data, sg::gen::flush_imu_data);

    EXPECT_NEAR(recovered.linear_acceleration.x, src.linear_acceleration.x, 1e-9);
    EXPECT_NEAR(recovered.linear_acceleration.y, src.linear_acceleration.y, 1e-9);
    EXPECT_NEAR(recovered.linear_acceleration.z, src.linear_acceleration.z, 1e-9);
    EXPECT_NEAR(recovered.angular_velocity.x,    src.angular_velocity.x,    1e-9);
    EXPECT_NEAR(recovered.angular_velocity.y,    src.angular_velocity.y,    1e-9);
    EXPECT_NEAR(recovered.angular_velocity.z,    src.angular_velocity.z,    1e-9);
    EXPECT_EQ(recovered.header.stamp.sec, src.header.stamp.sec);
    EXPECT_EQ(recovered.header.stamp.nanosec, src.header.stamp.nanosec);
}

TEST(ValidatorTest, ArrayRoundTrip_LaserScan_WithFilter)
{
    sensor_msgs::msg::LaserScan src;
    src.header.stamp.sec = 7;
    src.header.stamp.nanosec = 0;
    src.header.frame_id = "laser";
    src.angle_min = -static_cast<float>(M_PI);
    src.angle_max =  static_cast<float>(M_PI);
    src.angle_increment = 2.0f * static_cast<float>(M_PI) / LASER_BEAMS;
    src.range_min = 0.1f;
    src.range_max = 30.0f;
    src.ranges.resize(LASER_BEAMS);
    for (int i = 0; i < LASER_BEAMS; ++i) {
        // mix of valid and filtered-out values to exercise the array filter
        src.ranges[i] = (i % 8 == 0)
            ? std::numeric_limits<float>::infinity()       // dropped by filter
            : 1.0f + 0.25f * static_cast<float>(i);        // kept
    }

    std::vector<sg::msg::SmartData> sds;
    sg::gen::decompose_scan(src, 0.0, 0.0, 0.0, sds);

    // 4 of every 32 beams are filtered out → 28 SDs
    const std::size_t kept = LASER_BEAMS - LASER_BEAMS / 8;
    ASSERT_EQ(sds.size(), kept);
    for (const auto & sd : sds) {
        EXPECT_EQ(sd.unit, si_units::METER);
        EXPECT_NE(sd.origin_device % 8, 0u)
            << "filtered beam leaked through";
    }

    sg::gen::State_scan st;
    auto recovered = replay<sg::gen::State_scan, sensor_msgs::msg::LaserScan>(
        sds, st, sg::gen::feed_scan, sg::gen::flush_scan);

    // The reconstructor sizes the ranges vector to fit the largest dev seen.
    // The filter pattern (i%8==0 dropped) means the highest kept index is
    // LASER_BEAMS - 1 = 31 when LASER_BEAMS-1 is not a multiple of 8.
    ASSERT_EQ(recovered.ranges.size(), static_cast<std::size_t>(LASER_BEAMS));

    // Proposta §3.3: MSE entre leituras originais e reconstruídas tendente
    // a zero — computado apenas sobre os feixes que passaram pelo filtro
    // (índices i%8!=0); feixes filtrados não atravessam a SmartData e são
    // explicitamente lacunas.
    double mse_sum = 0.0;
    std::size_t mse_n = 0;
    for (int i = 0; i < LASER_BEAMS; ++i) {
        if (i % 8 == 0) {
            // dropped — recompose leaves the slot at the resize default (0)
            EXPECT_FLOAT_EQ(recovered.ranges[i], 0.0f)
                << "beam " << i << " should have been filtered";
            continue;
        }
        const double expected = 1.0 + 0.25 * i;
        const double got      = static_cast<double>(recovered.ranges[i]);
        const double err      = got - expected;
        mse_sum += err * err;
        ++mse_n;
        EXPECT_NEAR(got, expected, 1e-5)
            << "beam " << i << " mismatch";
    }
    ASSERT_GT(mse_n, 0u);
    const double mse = mse_sum / static_cast<double>(mse_n);
    GTEST_LOG_(INFO) << "LaserScan recompose MSE = " << mse
                     << " (over " << mse_n << " kept beams)";
    // float32 round-trip noise floor is ~1e-10 — anything above 1e-8 means
    // a real semantic loss, not numeric quantization.
    EXPECT_LT(mse, 1e-8) << "MSE deve tender a zero (proposta §3.3)";

    EXPECT_EQ(recovered.header.stamp.sec, src.header.stamp.sec);
    EXPECT_EQ(recovered.header.stamp.nanosec, src.header.stamp.nanosec);
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
