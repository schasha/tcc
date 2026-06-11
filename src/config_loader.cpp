#include "config_loader.hpp"
#include "si_units.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace semantic_gateway {

namespace {

// unit name table
const std::unordered_map<std::string, uint32_t> & unit_table()
{
    static const std::unordered_map<std::string, uint32_t> t = {
        {"meter",             si_units::METER},
        {"m",                 si_units::METER},
        {"kilogram",          si_units::KILOGRAM},
        {"kg",                si_units::KILOGRAM},
        {"second",            si_units::SECOND},
        {"s",                 si_units::SECOND},
        {"ampere",            si_units::AMPERE},
        {"A",                 si_units::AMPERE},
        {"kelvin",            si_units::KELVIN},
        {"K",                 si_units::KELVIN},
        {"mole",              si_units::MOLE},
        {"mol",               si_units::MOLE},
        {"candela",           si_units::CANDELA},
        {"cd",                si_units::CANDELA},
        {"radian",            si_units::RADIAN},
        {"rad",               si_units::RADIAN},
        {"meter_per_second",  si_units::METER_PER_SECOND},
        {"m_per_s",           si_units::METER_PER_SECOND},
        {"meter_per_sec_sq",  si_units::METER_PER_SEC_SQ},
        {"m_per_s2",          si_units::METER_PER_SEC_SQ},
        {"rad_per_sec",       si_units::RADIAN_PER_SECOND},
        {"radian_per_second", si_units::RADIAN_PER_SECOND},
        {"newton",            si_units::NEWTON},
        {"N",                 si_units::NEWTON},
        {"pascal",            si_units::PASCAL},
        {"Pa",                si_units::PASCAL},
        {"volt",              si_units::VOLT},
        {"V",                 si_units::VOLT},
    };
    return t;
}

Decomposition parse_decomposition(const std::string & s, const std::string & ctx)
{
    if (s == "scalar")    return Decomposition::Scalar;
    if (s == "array")     return Decomposition::Array;
    if (s == "composite") return Decomposition::Composite;
    throw std::runtime_error(
        "[" + ctx + "] unknown decomposition '" + s
        + "' (expected: scalar, array, composite)");
}

TransformKind parse_transform_kind(const std::string & s, const std::string & ctx)
{
    if (s == "none" || s.empty()) return TransformKind::None;
    if (s == "linear")            return TransformKind::Linear;
    throw std::runtime_error(
        "[" + ctx + "] unknown value_transform.kind '" + s
        + "' (expected: none, linear)");
}

FilterKind parse_filter_kind(const std::string & s, const std::string & ctx)
{
    if (s == "none" || s.empty()) return FilterKind::None;
    if (s == "finite")            return FilterKind::Finite;
    if (s == "finite_within")     return FilterKind::FiniteWithin;
    throw std::runtime_error(
        "[" + ctx + "] unknown filter.kind '" + s
        + "' (expected: none, finite, finite_within)");
}

// "/scan" -> "/smartdata/scan"; "/imu/data" -> "/smartdata/imu/data"
std::string default_output_topic(const std::string & input_topic)
{
    if (!input_topic.empty() && input_topic.front() == '/') {
        return "/smartdata" + input_topic;
    }
    return "/smartdata/" + input_topic;
}

SensorSpec parse_sensor(const YAML::Node & n, std::size_t idx)
{
    const std::string ctx = "sensor[" + std::to_string(idx) + "]";
    if (!n["topic"])        throw std::runtime_error("[" + ctx + "] missing 'topic'");
    if (!n["type"])         throw std::runtime_error("[" + ctx + "] missing 'type'");
    if (!n["decomposition"])throw std::runtime_error("[" + ctx + "] missing 'decomposition'");

    SensorSpec s;
    s.topic         = n["topic"].as<std::string>();
    s.type          = n["type"].as<std::string>();
    s.decomposition = parse_decomposition(n["decomposition"].as<std::string>(), ctx);

    if (n["stamp_field"])  s.stamp_field = n["stamp_field"].as<std::string>();
    if (n["frame_field"])  s.frame_field = n["frame_field"].as<std::string>();
    if (n["expiry_ms"]) {
        s.expiry_us = static_cast<uint64_t>(n["expiry_ms"].as<int64_t>()) * 1000ULL;
    }
    s.output_topic = n["output_topic"]
        ? n["output_topic"].as<std::string>()
        : default_output_topic(s.topic);

    switch (s.decomposition) {
    case Decomposition::Scalar: {
        if (!n["value_field"]) throw std::runtime_error("[" + ctx + "] scalar requires 'value_field'");
        if (!n["unit"])        throw std::runtime_error("[" + ctx + "] scalar requires 'unit'");
        s.value_field = n["value_field"].as<std::string>();
        s.unit        = unit_from_name(n["unit"].as<std::string>());
        if (n["value_transform"]) {
            const auto vt = n["value_transform"];
            s.value_transform.kind = parse_transform_kind(
                vt["kind"] ? vt["kind"].as<std::string>() : std::string{"linear"}, ctx);
            if (vt["a"]) s.value_transform.a = vt["a"].as<double>();
            if (vt["b"]) s.value_transform.b = vt["b"].as<double>();
        }
        break;
    }
    case Decomposition::Array: {
        if (!n["array_field"]) throw std::runtime_error("[" + ctx + "] array requires 'array_field'");
        if (!n["unit"])        throw std::runtime_error("[" + ctx + "] array requires 'unit'");
        s.array_field = n["array_field"].as<std::string>();
        s.unit        = unit_from_name(n["unit"].as<std::string>());
        if (n["filter"]) {
            const auto f = n["filter"];
            s.filter.kind = parse_filter_kind(
                f["kind"] ? f["kind"].as<std::string>() : std::string{"finite"}, ctx);
            if (f["min_field"]) s.filter.min_field = f["min_field"].as<std::string>();
            if (f["max_field"]) s.filter.max_field = f["max_field"].as<std::string>();
        }
        break;
    }
    case Decomposition::Composite: {
        if (!n["groups"] || !n["groups"].IsSequence()) {
            throw std::runtime_error("[" + ctx + "] composite requires 'groups' (sequence)");
        }
        for (std::size_t i = 0; i < n["groups"].size(); ++i) {
            const auto gn = n["groups"][i];
            const std::string gctx = ctx + ".group[" + std::to_string(i) + "]";
            if (!gn["unit"])   throw std::runtime_error("[" + gctx + "] missing 'unit'");
            if (!gn["fields"] || !gn["fields"].IsSequence()) {
                throw std::runtime_error("[" + gctx + "] missing 'fields' (sequence)");
            }
            CompositeGroup g;
            g.unit = unit_from_name(gn["unit"].as<std::string>());
            for (const auto & f : gn["fields"]) g.fields.push_back(f.as<std::string>());
            if (g.fields.empty()) {
                throw std::runtime_error("[" + gctx + "] 'fields' is empty");
            }
            s.groups.push_back(std::move(g));
        }
        if (s.groups.empty()) {
            throw std::runtime_error("[" + ctx + "] 'groups' is empty");
        }
        break;
    }
    }

    return s;
}

}  // namespace

uint32_t unit_from_name(const std::string & name)
{
    const auto & t = unit_table();
    const auto   it = t.find(name);
    if (it == t.end()) {
        throw std::runtime_error("unknown unit name '" + name + "'");
    }
    return it->second;
}

std::string unit_to_name(uint32_t unit)
{
    for (const auto & [k, v] : unit_table()) {
        if (v == unit) return k;
    }
    return si_units::to_string(unit);
}

GatewayConfig load_config_string(const std::string & yaml_text)
{
    const YAML::Node root = YAML::Load(yaml_text);
    GatewayConfig cfg;
    if (root["defaults"] && root["defaults"]["target_frame"]) {
        cfg.target_frame = root["defaults"]["target_frame"].as<std::string>();
    }
    if (!root["sensors"] || !root["sensors"].IsSequence()) {
        throw std::runtime_error("config missing 'sensors' sequence");
    }
    cfg.sensors.reserve(root["sensors"].size());
    for (std::size_t i = 0; i < root["sensors"].size(); ++i) {
        cfg.sensors.push_back(parse_sensor(root["sensors"][i], i));
    }
    return cfg;
}

GatewayConfig load_config_file(const std::string & yaml_path)
{
    std::ifstream in(yaml_path);
    if (!in) throw std::runtime_error("cannot open config file: " + yaml_path);
    std::ostringstream ss; ss << in.rdbuf();
    return load_config_string(ss.str());
}

}  // namespace semantic_gateway
