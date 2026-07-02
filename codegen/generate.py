#!/usr/bin/env python3
"""
generate.py — Semantic Gateway codegen.

Reads a sensors YAML (same schema the gateway used to parse at runtime) and
emits four C++ headers under <out>/generated/:

  parsed_decompose.hpp     one inline decompose_<safe_topic>() per sensor,
                           marker-delimited; baked unit codes, baked dev_base,
                           direct typed access (msg.foo.bar).

  parsed_dispatch.hpp      inline register_all(node, lookup_origin, on_sd)
                           creating one typed Subscription<T> per sensor and
                           calling the matching decompose_*().

  parsed_recompose.hpp     one inline feed_<safe_topic>() and flush_<safe_topic>()
                           per sensor — accumulates SmartData back into a typed
                           ROS message.

  parsed_sd_dispatch.hpp   inline Consumers struct + dispatch_sd(consumers, sd)
                           that routes an incoming SmartData to the right
                           feed_* based on (unit, dev_base range).

Discrimination at the bus consumer is by (unit, dev_base range): codegen
allocates a 16-bit dev_base slot per sensor sequentially, so two sensors
can share a unit without colliding. The dev_base in YAML is ignored.

Style follows SDAV's smartdata/include/transducers/parsed_transducers.h:
one file per kind, sections marked by "// --- Automatically Generated ... ---".

YAML is the source of truth at build time; adding a sensor = edit YAML + rebuild.

Usage:
    generate.py --yaml config/sensors.yaml --out build/generated
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from typing import Any

import yaml


# unit name → C++ symbol
UNIT_TO_SYMBOL = {
    "meter":             "si_units::METER",
    "m":                 "si_units::METER",
    "kilogram":          "si_units::KILOGRAM",
    "kg":                "si_units::KILOGRAM",
    "second":            "si_units::SECOND",
    "s":                 "si_units::SECOND",
    "ampere":            "si_units::AMPERE",
    "A":                 "si_units::AMPERE",
    "kelvin":            "si_units::KELVIN",
    "K":                 "si_units::KELVIN",
    "mole":              "si_units::MOLE",
    "mol":               "si_units::MOLE",
    "candela":           "si_units::CANDELA",
    "cd":                "si_units::CANDELA",
    "radian":            "si_units::RADIAN",
    "rad":               "si_units::RADIAN",
    "meter_per_second":  "si_units::METER_PER_SECOND",
    "m_per_s":           "si_units::METER_PER_SECOND",
    "meter_per_sec_sq":  "si_units::METER_PER_SEC_SQ",
    "m_per_s2":          "si_units::METER_PER_SEC_SQ",
    "rad_per_sec":       "si_units::RADIAN_PER_SECOND",
    "radian_per_second": "si_units::RADIAN_PER_SECOND",
    "newton":            "si_units::NEWTON",
    "N":                 "si_units::NEWTON",
    "pascal":            "si_units::PASCAL",
    "Pa":                "si_units::PASCAL",
    "volt":              "si_units::VOLT",
    "V":                 "si_units::VOLT",
}


# 16-bit dev space per sensor: up to 65535 devices each, max 65536 sensors.
DEV_RANGE_PER_SENSOR = 0x10000


def die(msg: str) -> None:
    print(f"codegen: {msg}", file=sys.stderr)
    sys.exit(1)


def unit_symbol(name: str, ctx: str) -> str:
    sym = UNIT_TO_SYMBOL.get(name)
    if sym is None:
        die(f"{ctx}: unknown unit '{name}'")
    return sym


def safe_ident(topic: str) -> str:
    """'/imu/data' → 'imu_data'; '/scan' → 'scan'."""
    s = re.sub(r"[^0-9A-Za-z]+", "_", topic).strip("_")
    if not s or s[0].isdigit():
        s = "_" + s
    return s


def pascal_to_snake(s: str) -> str:
    """'LaserScan' → 'laser_scan'."""
    s = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", s)
    s = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s)
    return s.lower()


def parse_type(type_id: str) -> tuple[str, str]:
    """'sensor_msgs/msg/LaserScan' → ('sensor_msgs/msg/laser_scan.hpp',
                                      'sensor_msgs::msg::LaserScan')."""
    parts = type_id.split("/")
    if len(parts) != 3:
        die(f"bad type id '{type_id}', expected pkg/msg/Type")
    pkg, sub, leaf = parts
    include = f"{pkg}/{sub}/{pascal_to_snake(leaf)}.hpp"
    cpp = f"{pkg}::{sub}::{leaf}"
    return include, cpp


def linear_apply(transform: dict[str, Any] | None, raw: str) -> str:
    """Apply linear transform a*v + b inline, or pass v through."""
    if not transform or transform.get("kind", "none") == "none":
        return raw
    if transform.get("kind") != "linear":
        die(f"unknown transform kind '{transform.get('kind')}'")
    a = float(transform.get("a", 1.0))
    b = float(transform.get("b", 0.0))
    if a == 1.0 and b == 0.0:
        return raw
    if a == 1.0:
        return f"({raw}) + {b!r}"
    if b == 0.0:
        return f"{a!r} * ({raw})"
    return f"{a!r} * ({raw}) + {b!r}"


def linear_inverse(transform: dict[str, Any] | None, raw: str) -> str:
    """Invert linear transform — used by recompose to undo unit conversion."""
    if not transform or transform.get("kind", "none") == "none":
        return raw
    if transform.get("kind") != "linear":
        die(f"unknown transform kind '{transform.get('kind')}'")
    a = float(transform.get("a", 1.0))
    b = float(transform.get("b", 0.0))
    if a == 1.0 and b == 0.0:
        return raw
    if a == 1.0:
        return f"({raw}) - {b!r}"
    if b == 0.0:
        return f"({raw}) / {a!r}"
    return f"(({raw}) - {b!r}) / {a!r}"


def stamp_us_block(stamp_field: str) -> str:
    return (
        f"uint64_t(msg.{stamp_field}.sec) * 1'000'000ull + "
        f"uint64_t(msg.{stamp_field}.nanosec) / 1000ull"
    )


def allocate_dev_bases(sensors: list[dict[str, Any]]) -> None:
    """Assign sensor.dev_base = i * DEV_RANGE_PER_SENSOR and propagate group
    bases for composite sensors. Overrides any value in YAML."""
    for i, s in enumerate(sensors):
        s["_dev_base"]  = i * DEV_RANGE_PER_SENSOR
        s["_dev_range"] = DEV_RANGE_PER_SENSOR
        if s.get("decomposition") == "composite":
            next_off = 0
            for g in (s.get("groups") or []):
                g["_dev_base"] = s["_dev_base"] + next_off
                next_off += len(g.get("fields") or [])


# ─── decompose emitters ──────────────────────────────────────────────────────


def emit_decompose_scalar(name: str, cpp_type: str, spec: dict[str, Any]) -> str:
    unit = unit_symbol(spec["unit"], f"sensor[{name}].unit")
    expiry_us = int(float(spec.get("expiry_ms", 0)) * 1000)
    dev_base = spec["_dev_base"]
    value_field = spec["value_field"]
    stamp_field = spec.get("stamp_field", "header.stamp")
    value_expr = linear_apply(spec.get("value_transform"),
                               f"msg.{value_field}")
    return f"""\
// --- Automatically Generated Decompose: {name} (scalar) ---
inline void decompose_{name}(
    const {cpp_type} & msg,
    double ox, double oy, double oz,
    std::vector<msg::SmartData> & out)
{{
    constexpr uint32_t UNIT      = {unit};
    constexpr uint32_t DEV_BASE  = {dev_base}u;
    constexpr uint64_t EXPIRY_US = {expiry_us}ull;

    const uint64_t t_us = {stamp_us_block(stamp_field)};
    const double   v    = {value_expr};

    out.push_back(make_sd(UNIT, v, ox, oy, oz, DEV_BASE, t_us, EXPIRY_US));
}}
"""


def emit_decompose_array(name: str, cpp_type: str, spec: dict[str, Any]) -> str:
    unit = unit_symbol(spec["unit"], f"sensor[{name}].unit")
    expiry_us = int(float(spec.get("expiry_ms", 0)) * 1000)
    dev_base = spec["_dev_base"]
    array_field = spec["array_field"]
    stamp_field = spec.get("stamp_field", "header.stamp")
    value_expr = linear_apply(spec.get("value_transform"), "raw")

    filt = spec.get("filter") or {"kind": "none"}
    fkind = filt.get("kind", "none")

    if fkind == "none":
        filter_block = ""
    elif fkind == "finite":
        filter_block = "        if (!std::isfinite(raw)) continue;\n"
    elif fkind == "finite_within":
        lo_field = filt["min_field"]
        hi_field = filt["max_field"]
        filter_block = f"""\
        if (!std::isfinite(raw)) continue;
        if (raw < msg.{lo_field} || raw > msg.{hi_field}) continue;
"""
    else:
        die(f"sensor[{name}]: unknown filter.kind '{fkind}'")

    return f"""\
// --- Automatically Generated Decompose: {name} (array) ---
inline void decompose_{name}(
    const {cpp_type} & msg,
    double ox, double oy, double oz,
    std::vector<msg::SmartData> & out)
{{
    constexpr uint32_t UNIT      = {unit};
    constexpr uint32_t DEV_BASE  = {dev_base}u;
    constexpr uint64_t EXPIRY_US = {expiry_us}ull;

    const uint64_t t_us = {stamp_us_block(stamp_field)};
    const std::size_t n = msg.{array_field}.size();
    out.reserve(out.size() + n);

    for (uint32_t i = 0; i < n; ++i) {{
        const double raw = static_cast<double>(msg.{array_field}[i]);
{filter_block}\
        const double v = {value_expr};
        out.push_back(make_sd(UNIT, v, ox, oy, oz,
                              DEV_BASE + i, t_us, EXPIRY_US));
    }}
}}
"""


def emit_decompose_composite(name: str, cpp_type: str,
                              spec: dict[str, Any]) -> str:
    expiry_us = int(float(spec.get("expiry_ms", 0)) * 1000)
    stamp_field = spec.get("stamp_field", "header.stamp")

    groups = spec.get("groups") or []
    if not groups:
        die(f"sensor[{name}]: composite needs non-empty 'groups'")

    total = sum(len(g.get("fields") or []) for g in groups)

    body_lines: list[str] = []
    for gi, g in enumerate(groups):
        u = unit_symbol(g["unit"], f"sensor[{name}].groups[{gi}].unit")
        fields = g.get("fields") or []
        dev_base = g["_dev_base"]
        body_lines.append(f"    // group {gi}: unit={g['unit']} "
                          f"dev_base={dev_base}")
        for fi, fpath in enumerate(fields):
            body_lines.append(
                f"    out.push_back(make_sd({u}, msg.{fpath}, "
                f"ox, oy, oz, {dev_base + fi}u, t_us, EXPIRY_US));"
            )

    body = "\n".join(body_lines)

    return f"""\
// --- Automatically Generated Decompose: {name} (composite) ---
inline void decompose_{name}(
    const {cpp_type} & msg,
    double ox, double oy, double oz,
    std::vector<msg::SmartData> & out)
{{
    constexpr uint64_t EXPIRY_US = {expiry_us}ull;
    const uint64_t t_us = {stamp_us_block(stamp_field)};
    out.reserve(out.size() + {total}u);
{body}
}}
"""


# ─── recompose emitters ──────────────────────────────────────────────────────


def stamp_assignment_block(target: str, var: str = "sd.origin_time") -> str:
    """Write origin_time (µs) into a header.stamp field.

    target is the C++ lvalue 'msg.<stamp_field>'.
    """
    return (
        f"    {target}.sec = static_cast<int32_t>({var} / 1'000'000ull);\n"
        f"    {target}.nanosec = static_cast<uint32_t>(({var} % 1'000'000ull) * 1000ull);"
    )


def emit_recompose_scalar(name: str, cpp_type: str, spec: dict[str, Any]) -> str:
    value_field = spec["value_field"]
    stamp_field = spec.get("stamp_field", "header.stamp")
    inverse = linear_inverse(spec.get("value_transform"), "sd.value")
    return f"""\
// --- Automatically Generated Recompose: {name} (scalar) ---
struct State_{name} {{
    bool dummy {{false}};  // scalar has no state — flushed immediately
}};

inline void feed_{name}(State_{name} &,
                        const msg::SmartData & sd,
                        std::optional<{cpp_type}> & out)
{{
    {cpp_type} m;
{stamp_assignment_block(f"m.{stamp_field}")}
    m.{value_field} = static_cast<decltype(m.{value_field})>({inverse});
    out = std::move(m);
}}

inline void flush_{name}(State_{name} &, std::optional<{cpp_type}> &) {{}}
"""


def emit_recompose_array(name: str, cpp_type: str, spec: dict[str, Any]) -> str:
    dev_base = spec["_dev_base"]
    array_field = spec["array_field"]
    stamp_field = spec.get("stamp_field", "header.stamp")
    inverse = linear_inverse(spec.get("value_transform"), "sd.value")
    return f"""\
// --- Automatically Generated Recompose: {name} (array) ---
struct State_{name} {{
    bool         has {{false}};
    uint64_t     origin_time {{0}};
    {cpp_type}   msg {{}};
    std::size_t  count {{0}};
}};

inline void feed_{name}(State_{name} & s,
                        const msg::SmartData & sd,
                        std::optional<{cpp_type}> & out)
{{
    constexpr uint32_t DEV_BASE = {dev_base}u;

    if (s.has && s.origin_time != sd.origin_time) {{
        out = std::move(s.msg);
        s = State_{name}{{}};
    }}
    if (!s.has) {{
        s.has         = true;
        s.origin_time = sd.origin_time;
{stamp_assignment_block(f"s.msg.{stamp_field}", "s.origin_time")}
    }}
    const uint32_t local = sd.origin_device - DEV_BASE;
    if (local >= s.msg.{array_field}.size()) {{
        s.msg.{array_field}.resize(local + 1, 0);
    }}
    s.msg.{array_field}[local] =
        static_cast<typename decltype(s.msg.{array_field})::value_type>({inverse});
    ++s.count;
}}

inline void flush_{name}(State_{name} & s, std::optional<{cpp_type}> & out)
{{
    if (s.has) {{
        out = std::move(s.msg);
        s = State_{name}{{}};
    }}
}}
"""


def emit_recompose_composite(name: str, cpp_type: str,
                              spec: dict[str, Any]) -> str:
    stamp_field = spec.get("stamp_field", "header.stamp")

    groups = spec.get("groups") or []
    cases: list[str] = []
    expected = 0
    for g in groups:
        for fi, fpath in enumerate(g.get("fields") or []):
            dev = g["_dev_base"] + fi
            cases.append(
                f"        case {dev}u: s.msg.{fpath} = sd.value; break;"
            )
            expected += 1
    case_block = "\n".join(cases)

    return f"""\
// --- Automatically Generated Recompose: {name} (composite) ---
struct State_{name} {{
    bool         has {{false}};
    uint64_t     origin_time {{0}};
    {cpp_type}   msg {{}};
    std::size_t  count {{0}};
}};

inline void feed_{name}(State_{name} & s,
                        const msg::SmartData & sd,
                        std::optional<{cpp_type}> & out)
{{
    constexpr std::size_t EXPECTED = {expected}u;

    if (s.has && s.origin_time != sd.origin_time) {{
        out = std::move(s.msg);
        s = State_{name}{{}};
    }}
    if (!s.has) {{
        s.has         = true;
        s.origin_time = sd.origin_time;
{stamp_assignment_block(f"s.msg.{stamp_field}", "s.origin_time")}
    }}
    switch (sd.origin_device) {{
{case_block}
        default: break;
    }}
    ++s.count;
    if (s.count >= EXPECTED) {{
        out = std::move(s.msg);
        s = State_{name}{{}};
    }}
}}

inline void flush_{name}(State_{name} & s, std::optional<{cpp_type}> & out)
{{
    if (s.has) {{
        out = std::move(s.msg);
        s = State_{name}{{}};
    }}
}}
"""


def default_output_topic(input_topic: str) -> str:
    if input_topic.startswith("/"):
        return "/smartdata" + input_topic
    return "/smartdata/" + input_topic


def default_recovered_topic(input_topic: str) -> str:
    """Topic where the reconstructed ROS message is republished."""
    if input_topic.startswith("/"):
        return input_topic + "_recovered"
    return "/" + input_topic + "_recovered"


# ─── file assembly ───────────────────────────────────────────────────────────


def emit_parsed_decompose(sensors: list[dict[str, Any]],
                          source_yaml: str) -> str:
    includes: list[str] = []
    seen_includes: set[str] = set()
    sections: list[str] = []

    for s in sensors:
        topic = s["topic"]
        name = safe_ident(topic)
        include, cpp_type = parse_type(s["type"])
        if include not in seen_includes:
            includes.append(include)
            seen_includes.add(include)

        kind = s["decomposition"]
        if kind == "scalar":
            sections.append(emit_decompose_scalar(name, cpp_type, s))
        elif kind == "array":
            sections.append(emit_decompose_array(name, cpp_type, s))
        elif kind == "composite":
            sections.append(emit_decompose_composite(name, cpp_type, s))
        else:
            die(f"sensor[{topic}]: unknown decomposition '{kind}'")

    include_block = "\n".join(f'#include "{i}"' for i in includes)
    body = "\n".join(sections)

    return f"""\
// AUTO-GENERATED by codegen/generate.py — do not edit.
// Source: {source_yaml}
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

{include_block}

#include "semantic_gateway/msg/smart_data.hpp"
#include "sd_codec.hpp"
#include "si_units.hpp"

namespace semantic_gateway::gen {{

using semantic_gateway::make_sd;
namespace msg = semantic_gateway::msg;

{body}
}}  // namespace semantic_gateway::gen
"""


def emit_parsed_dispatch(sensors: list[dict[str, Any]],
                         source_yaml: str) -> str:
    cases: list[str] = []
    for s in sensors:
        topic = s["topic"]
        name = safe_ident(topic)
        _, cpp_type = parse_type(s["type"])
        output_topic = s.get("output_topic") or default_output_topic(topic)
        stamp_field = s.get("stamp_field", "header.stamp")
        frame_field = s.get("frame_field", "header.frame_id")
        cases.append(f"""\
    // --- Automatically Generated Dispatch: {name} ---
    {{
        const std::string out_topic = "{output_topic}";
        auto sub = node->create_subscription<{cpp_type}>(
            "{topic}", rclcpp::SensorDataQoS(),
            [lookup_origin, on_sd, out_topic]
            ({cpp_type}::ConstSharedPtr m) {{
                auto origin = lookup_origin(m->{frame_field}, m->{stamp_field});
                std::vector<msg::SmartData> out;
                decompose_{name}(*m, std::get<0>(origin),
                                       std::get<1>(origin),
                                       std::get<2>(origin), out);
                for (const auto & sd : out) on_sd(out_topic, sd);
            }});
        subs.handles.push_back(sub);
    }}""")

    body = "\n".join(cases)

    return f"""\
// AUTO-GENERATED by codegen/generate.py — do not edit.
// Source: {source_yaml}
#pragma once

#include <functional>
#include <string>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "parsed_decompose.hpp"

namespace semantic_gateway::gen {{

using FrameLookup = std::function<std::tuple<double,double,double>(
    const std::string & frame,
    const builtin_interfaces::msg::Time & stamp)>;

using OnSmartData = std::function<void(
    const std::string & topic, const msg::SmartData & sd)>;

struct Subs {{
    std::vector<rclcpp::SubscriptionBase::SharedPtr> handles;
}};

inline Subs register_all(rclcpp::Node * node,
                         FrameLookup   lookup_origin,
                         OnSmartData   on_sd)
{{
    Subs subs;
{body}
    return subs;
}}

}}  // namespace semantic_gateway::gen
"""


def emit_parsed_recompose(sensors: list[dict[str, Any]],
                           source_yaml: str) -> str:
    includes: list[str] = []
    seen_includes: set[str] = set()
    sections: list[str] = []

    for s in sensors:
        topic = s["topic"]
        name = safe_ident(topic)
        include, cpp_type = parse_type(s["type"])
        if include not in seen_includes:
            includes.append(include)
            seen_includes.add(include)

        kind = s["decomposition"]
        if kind == "scalar":
            sections.append(emit_recompose_scalar(name, cpp_type, s))
        elif kind == "array":
            sections.append(emit_recompose_array(name, cpp_type, s))
        elif kind == "composite":
            sections.append(emit_recompose_composite(name, cpp_type, s))

    include_block = "\n".join(f'#include "{i}"' for i in includes)
    body = "\n".join(sections)

    return f"""\
// AUTO-GENERATED by codegen/generate.py — do not edit.
// Source: {source_yaml}
#pragma once

#include <cstdint>
#include <optional>
#include <utility>

{include_block}

#include "semantic_gateway/msg/smart_data.hpp"

namespace semantic_gateway::gen {{

namespace msg = semantic_gateway::msg;

{body}
}}  // namespace semantic_gateway::gen
"""


def emit_parsed_sd_dispatch(sensors: list[dict[str, Any]],
                             source_yaml: str) -> str:
    # Consumers struct: state + on_msg callback per sensor.
    fields: list[str] = []
    range_consts: list[str] = []
    dispatch_branches: list[str] = []
    drain_blocks: list[str] = []

    for s in sensors:
        topic = s["topic"]
        name = safe_ident(topic)
        _, cpp_type = parse_type(s["type"])
        dev_base = s["_dev_base"]
        dev_range = s["_dev_range"]
        unit_for_check = None
        # for unit sanity check (composite spans multiple units → skip)
        if s["decomposition"] in ("scalar", "array"):
            unit_for_check = unit_symbol(s["unit"], f"sensor[{name}].unit")

        fields.append(f"    State_{name} st_{name};")
        fields.append(
            f"    std::function<void(const {cpp_type} &)> on_{name};"
        )
        range_consts.append(
            f"constexpr uint32_t DEV_BASE_{name}  = {dev_base}u;"
        )
        range_consts.append(
            f"constexpr uint32_t DEV_RANGE_{name} = {dev_range}u;"
        )

        unit_guard = (
            f"sd.unit == {unit_for_check} && "
            if unit_for_check else ""
        )
        dispatch_branches.append(f"""\
    if ({unit_guard}sd.origin_device >= DEV_BASE_{name} &&
        sd.origin_device <  DEV_BASE_{name} + DEV_RANGE_{name}) {{
        std::optional<{cpp_type}> out;
        feed_{name}(c.st_{name}, sd, out);
        if (out && c.on_{name}) c.on_{name}(*out);
        return;
    }}""")

        drain_blocks.append(f"""\
    {{
        std::optional<{cpp_type}> out;
        flush_{name}(c.st_{name}, out);
        if (out && c.on_{name}) c.on_{name}(*out);
    }}""")

    fields_block = "\n".join(fields)
    range_block = "\n".join(range_consts)
    dispatch_block = "\n".join(dispatch_branches)
    drain_block = "\n".join(drain_blocks)

    return f"""\
// AUTO-GENERATED by codegen/generate.py — do not edit.
// Source: {source_yaml}
#pragma once

#include <cstdint>
#include <functional>
#include <optional>

#include "semantic_gateway/msg/smart_data.hpp"
#include "si_units.hpp"

#include "parsed_recompose.hpp"

namespace semantic_gateway::gen {{

namespace msg = semantic_gateway::msg;

{range_block}

struct Consumers {{
{fields_block}
}};

inline void dispatch_sd(Consumers & c, const msg::SmartData & sd)
{{
{dispatch_block}
}}

inline void drain(Consumers & c)
{{
{drain_block}
}}

}}  // namespace semantic_gateway::gen
"""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", required=True, help="path to sensors.yaml")
    ap.add_argument("--out",  required=True, help="output directory")
    args = ap.parse_args()

    if not os.path.isfile(args.yaml):
        die(f"yaml not found: {args.yaml}")

    with open(args.yaml, "r") as f:
        cfg = yaml.safe_load(f)

    sensors = cfg.get("sensors") or []
    if not sensors:
        die("no sensors in yaml")

    allocate_dev_bases(sensors)

    os.makedirs(args.out, exist_ok=True)

    outputs = {
        "parsed_decompose.hpp":   emit_parsed_decompose(sensors, args.yaml),
        "parsed_dispatch.hpp":    emit_parsed_dispatch(sensors, args.yaml),
        "parsed_recompose.hpp":   emit_parsed_recompose(sensors, args.yaml),
        "parsed_sd_dispatch.hpp": emit_parsed_sd_dispatch(sensors, args.yaml),
    }

    for fname, content in outputs.items():
        with open(os.path.join(args.out, fname), "w") as f:
            f.write(content)

    print(f"codegen: wrote {len(outputs)} headers to {args.out} "
          f"({len(sensors)} sensors)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
