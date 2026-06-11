#pragma once

// config_loader.hpp — YAML → GatewayConfig
//
// parses a sensors.yaml file into a typed configuration. throws std::runtime_error
// on malformed input with a message naming the wrong sensor.
// ──────────────────────────────────────────────────────────────────────────────

#include <string>

#include "spec.hpp"

namespace semantic_gateway {

// parse a YAML file
GatewayConfig load_config_file(const std::string & yaml_path);

// parse a YAML string (used by tests)
GatewayConfig load_config_string(const std::string & yaml_text);

// resolve a unit name (e.g. "kelvin", "meter_per_sec_sq") into a 32-bit code.
// throws std::runtime_error on unknown names.
uint32_t unit_from_name(const std::string & name);

// inverse: readable name for a unit code (used by tests and logs).
std::string unit_to_name(uint32_t unit);

}  // namespace semantic_gateway
