#pragma once

// sd_codec.hpp — SmartData factory used by generated decompose_*().

#include <cstdint>

#include "semantic_gateway/msg/smart_data.hpp"

namespace semantic_gateway {

inline msg::SmartData make_sd(
    uint32_t unit, double value,
    double ox, double oy, double oz,
    uint32_t dev, uint64_t t_us, uint64_t expiry_us)
{
    msg::SmartData sd;
    sd.unit          = unit;
    sd.value         = value;
    sd.origin_x      = ox;
    sd.origin_y      = oy;
    sd.origin_z      = oz;
    sd.origin_device = dev;
    sd.origin_time   = t_us;
    sd.expiry        = expiry_us;
    sd.mac.fill(0);
    return sd;
}

}  // namespace semantic_gateway
