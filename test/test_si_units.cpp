// test_si_units.cpp — unit tests for the 32-bit dimensional SI encoding.
//
// Pins the bit layout from Fröhlich (2018) Fig. 1 and the constants exposed
// in si_units.hpp. Failing here means the wire-level unit code is wrong,
// which would break interop with every SmartData consumer.

#include <gtest/gtest.h>

#include "si_units.hpp"

TEST(SiUnits, MeterIsPureLengthExponent)
{
    const auto d = si_units::decode(si_units::METER);
    EXPECT_EQ(d.type, si_units::Type::SI);
    EXPECT_EQ(d.numeric, si_units::Numeric::Float64);
    EXPECT_EQ(d.modifier, si_units::Modifier::Direct);
    EXPECT_EQ(d.m,   1);
    EXPECT_EQ(d.kg,  0);
    EXPECT_EQ(d.s,   0);
    EXPECT_EQ(d.A,   0);
    EXPECT_EQ(d.K,   0);
    EXPECT_EQ(d.mol, 0);
    EXPECT_EQ(d.cd,  0);
    EXPECT_EQ(d.rad, 0);
    EXPECT_EQ(d.sr,  0);
}

TEST(SiUnits, MeterPerSecSqEncodesAccel)
{
    const auto d = si_units::decode(si_units::METER_PER_SEC_SQ);
    EXPECT_EQ(d.m,  1);
    EXPECT_EQ(d.s, -2);
    EXPECT_EQ(d.kg, 0);
    EXPECT_EQ(d.A,  0);
}

TEST(SiUnits, RadianPerSecondEncodesAngularVel)
{
    const auto d = si_units::decode(si_units::RADIAN_PER_SECOND);
    EXPECT_EQ(d.rad, 1);
    EXPECT_EQ(d.s,  -1);
    EXPECT_EQ(d.m,   0);
    EXPECT_EQ(d.kg,  0);
}

TEST(SiUnits, VoltMatchesPaperFig1)
{
    // Volt = kg·m²·s⁻³·A⁻¹
    const auto d = si_units::decode(si_units::VOLT);
    EXPECT_EQ(d.kg,  1);
    EXPECT_EQ(d.m,   2);
    EXPECT_EQ(d.s,  -3);
    EXPECT_EQ(d.A,  -1);
}

TEST(SiUnits, ToStringRendersDimensions)
{
    EXPECT_EQ(si_units::to_string(si_units::METER), "m");
    EXPECT_NE(si_units::to_string(si_units::VOLT).find("kg"), std::string::npos);
    EXPECT_NE(si_units::to_string(si_units::VOLT).find("m"),  std::string::npos);
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
