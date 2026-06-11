#pragma once

// spec.hpp — in-memory representation of the YAML sensor configuration.
//
// each `SensorSpec` describes how to translate one ROS topic into one or more
// SmartData topics.
//
//  scalar     — one SmartData per ROS message
//  array      — N SmartData per ROS message, dev = element index
//  composite  — several groups of fields, each group with its own unit
//
// see config/sensors.yaml for the canonical example and docs/idl_mapping.md
// for semantics.

#include <cstdint>
#include <string>
#include <vector>

namespace semantic_gateway {

enum class Decomposition : uint8_t { Scalar, Array, Composite };

enum class TransformKind : uint8_t { None, Linear };

struct ValueTransform {
    TransformKind kind{TransformKind::None};
    double a{1.0};
    double b{0.0};

    double apply(double v) const noexcept
    {
        return kind == TransformKind::Linear ? a * v + b : v;
    }
};

enum class FilterKind : uint8_t {
    None,
    Finite,         // accept only finite (no inf nor NaN)
    FiniteWithin    // finite AND within [min_field, max_field] read from the message
};

struct Filter {
    FilterKind  kind{FilterKind::None};
    std::string min_field;   //used by FiniteWithin
    std::string max_field;
};

struct CompositeGroup {
    uint32_t                 unit{0};
    std::vector<std::string> fields;
};

struct SensorSpec {
    // required for every sensor
    std::string   topic;                   // input ROS topic
    std::string   type;                    // "sensor_msgs/msg/LaserScan" etc.
    std::string   stamp_field{"header.stamp"};
    std::string   frame_field{"header.frame_id"};
    Decomposition decomposition{Decomposition::Scalar};
    uint64_t      expiry_us{0};
    std::string   output_topic;            // SmartData topic; derived if empty

    // scalar
    std::string    value_field;
    ValueTransform value_transform;
    uint32_t       unit{0};

    // array
    std::string array_field;
    Filter      filter;

    // composite
    std::vector<CompositeGroup> groups;
};

struct GatewayConfig {
    std::string             target_frame{"map"};
    std::vector<SensorSpec> sensors;
};

}  // namespace semantic_gateway
