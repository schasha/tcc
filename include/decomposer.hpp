#pragma once

// decomposer.hpp — generic ROS → SmartData decomposition.
//
// `decompose(spec, mv, origin, expiry)` reads fields from the typed buffer
// inside `mv` according to `spec` and produces the corresponding sequence of
// SmartData.

#include <cmath>
#include <cstdint>
#include <vector>

#include "field_access.hpp"
#include "spec.hpp"
#include "semantic_gateway/msg/smart_data.hpp"

namespace semantic_gateway {

inline bool is_valid(const msg::SmartData & sd, uint64_t now_us) noexcept
{
    return now_us < (sd.origin_time + sd.expiry);
}

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

inline std::vector<msg::SmartData> decompose(
    const SensorSpec & spec, const MessageView & mv,
    double ox, double oy, double oz)
{
    const uint64_t t_us = mv.read_time_us(spec.stamp_field);
    std::vector<msg::SmartData> out;

    switch (spec.decomposition) {

    case Decomposition::Scalar: {
        const double v = spec.value_transform.apply(
            mv.read_double(spec.value_field));
        out.reserve(1);
        out.push_back(make_sd(spec.unit, v, ox, oy, oz, 0, t_us, spec.expiry_us));
        break;
    }

    case Decomposition::Array: {
        const std::size_t n = mv.array_size(spec.array_field);
        out.reserve(n);

        double lo = -std::numeric_limits<double>::infinity();
        double hi =  std::numeric_limits<double>::infinity();
        if (spec.filter.kind == FilterKind::FiniteWithin) {
            lo = mv.read_double(spec.filter.min_field);
            hi = mv.read_double(spec.filter.max_field);
        }

        for (std::size_t i = 0; i < n; ++i) {
            const double v = mv.read_array_double(spec.array_field, i);
            if (spec.filter.kind != FilterKind::None) {
                if (!std::isfinite(v))             continue;
                if (spec.filter.kind == FilterKind::FiniteWithin) {
                    if (v < lo || v > hi) continue;
                }
            }
            out.push_back(make_sd(spec.unit, spec.value_transform.apply(v),
                                   ox, oy, oz,
                                   static_cast<uint32_t>(i),
                                   t_us, spec.expiry_us));
        }
        break;
    }

    case Decomposition::Composite: {
        std::size_t total = 0;
        for (const auto & g : spec.groups) total += g.fields.size();
        out.reserve(total);
        for (const auto & g : spec.groups) {
            for (uint32_t i = 0; i < g.fields.size(); ++i) {
                const double v = mv.read_double(g.fields[i]);
                out.push_back(make_sd(g.unit, v, ox, oy, oz, i, t_us, spec.expiry_us));
            }
        }
        break;
    }
    }

    return out;
}

}  // namespace semantic_gateway
