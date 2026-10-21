#!/usr/bin/env python3
"""
generate.py — Semantic Gateway codegen.

Reads the sensors/actuators YAML and emits three C++ headers under <out>/:

  parsed_transducers.hpp   one Transducer class per (ROS topic x physical
                           quantity), in the sense of Fröhlich (2018) §2.1:
                           it declares UNIT, TYPE (SENSOR/ACTUATOR) and
                           `active`, and its instances are told apart by
                           `dev`. Sensor transducers observe a ROS topic (the
                           message arrival is their "interrupt") and read
                           their field with direct, typed access. Actuator
                           transducers write their field of the setpoint
                           message and publish it.

  parsed_gateway.hpp       class Gateway: the ROS subscriptions, the topic
                           sources/sinks and one Responsive_SmartData per
                           transducer instance (ADVERTISED sensors,
                           COMMANDED actuators).

  parsed_proxies.hpp       class Proxies: the remote side. One
                           Interested_SmartData per sensor transducer class
                           rebuilds the ROS messages from the responses;
                           one per actuator instance turns ROS messages on
                           <topic>_command into SmartData commands.

Device numbering: each transducer class gets a disjoint 16-bit block of
device ids (class k -> [k * 2^16, (k + 1) * 2^16)); its instances are
DEV_BASE + index (vector element, field of a group). Two classes sharing a
unit on the same node therefore never share a device id.

Style follows SDAV's smartdata/include/transducers/parsed_transducers.h:
one generated section per transducer, marked "// --- Automatically
Generated ... ---".

Usage:
    generate.py --yaml config/sensors.yaml --out build/generated
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from typing import Any

import yaml


# unit name -> C++ symbol
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

MODES = {
    "private":    "sd::PRIVATE",
    "advertised": "sd::ADVERTISED",
}

# 16-bit device block per transducer class
DEV_RANGE = 0x10000


def die(msg: str) -> None:
    print(f"codegen: {msg}", file=sys.stderr)
    sys.exit(1)


def unit_symbol(name: Any, ctx: str) -> str:
    # a raw 32-bit unit code lets imports take units without a name here
    if isinstance(name, int) and not isinstance(name, bool):
        if not 0 < name <= 0xFFFFFFFF:
            die(f"{ctx}: unit code {name:#x} is not a 32-bit value")
        return f"{name:#010x}u"
    sym = UNIT_TO_SYMBOL.get(name)
    if sym is None:
        die(f"{ctx}: unknown unit '{name}'")
    return sym


def safe_ident(text: str) -> str:
    """'/imu/data' -> 'imu_data'."""
    s = re.sub(r"[^0-9A-Za-z]+", "_", text).strip("_")
    if not s or s[0].isdigit():
        s = "_" + s
    return s


def pascal(name: str) -> str:
    """'imu_data_linear_acceleration' -> 'ImuDataLinearAcceleration'."""
    return "".join(p[:1].upper() + p[1:] for p in name.split("_") if p)


def pascal_to_snake(s: str) -> str:
    s = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", s)
    s = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s)
    return s.lower()


def parse_type(type_id: str) -> tuple[str, str]:
    """'sensor_msgs/msg/LaserScan' -> (include, C++ type)."""
    parts = type_id.split("/")
    if len(parts) != 3:
        die(f"bad type id '{type_id}', expected pkg/msg/Type")
    pkg, sub, leaf = parts
    return f"{pkg}/{sub}/{pascal_to_snake(leaf)}.hpp", f"{pkg}::{sub}::{leaf}"


def transform_exprs(transform: dict[str, Any] | None) -> tuple[str, str]:
    """(forward, inverse) C++ expressions over `v`: ROS value <-> SI value."""
    if not transform or transform.get("kind", "none") == "none":
        return "v", "v"
    if transform.get("kind") != "linear":
        die(f"unknown transform kind '{transform.get('kind')}'")
    a = float(transform.get("a", 1.0))
    b = float(transform.get("b", 0.0))
    if a == 0.0:
        die("linear transform with a = 0 is not invertible")
    if a == 1.0 and b == 0.0:
        return "v", "v"
    if a == 1.0:
        return f"v + {b!r}", f"v - {b!r}"
    if b == 0.0:
        return f"{a!r} * v", f"v / {a!r}"
    return f"{a!r} * v + {b!r}", f"(v - {b!r}) / {a!r}"


def cstr(text: str) -> str:
    """C++ string literal."""
    return json.dumps(text)


def ms_to_us(value: Any, ctx: str) -> int:
    us = int(round(float(value) * 1000))
    if us < 0:
        die(f"{ctx}: must be non-negative")
    return us


# ─── model ───────────────────────────────────────────────────────────────────


def build_model(cfg: dict[str, Any]) -> tuple[list[dict], list[dict]]:
    """Returns (topics, transducers). Each transducer refers to its topic."""
    topics: list[dict] = []
    transducers: list[dict] = []

    entries = [(s, False) for s in (cfg.get("sensors") or [])]
    entries += [(a, True) for a in (cfg.get("actuators") or [])]
    if not entries:
        die("no sensors or actuators in yaml")

    seen_topics: set[str] = set()
    for spec, actuator in entries:
        topic = spec.get("topic") or die("entry without 'topic'")
        if topic in seen_topics:
            die(f"topic '{topic}' declared twice")
        seen_topics.add(topic)
        include, cpp = parse_type(spec.get("type") or die(f"{topic}: missing 'type'"))
        kind = spec.get("decomposition")
        ctx = f"{'actuator' if actuator else 'sensor'}[{topic}]"

        expiry = ms_to_us(spec.get("expiry_ms", 0), f"{ctx}.expiry_ms")
        period = ms_to_us(spec.get("period_ms", 0), f"{ctx}.period_ms")
        if period > 0 and 0 < expiry < period:
            die(f"{ctx}: expiry_ms smaller than period_ms is invalid")

        if actuator:
            mode = "sd::COMMANDED"
            if kind not in ("scalar", "composite"):
                die(f"{ctx}: actuators support scalar or composite decomposition")
        else:
            mode_name = spec.get("mode", "advertised")
            mode = MODES.get(mode_name) or die(f"{ctx}: unknown mode '{mode_name}'")

        t = {
            "topic":       topic,
            "name":        safe_ident(topic),
            "include":     include,
            "cpp":         cpp,
            "actuator":    actuator,
            "kind":        kind,
            "expiry":      expiry,
            "period":      period,
            "mode":        mode,
            "stamp_field": spec.get("stamp_field", None if actuator else "header.stamp"),
            "frame_field": spec.get("frame_field", "header.frame_id"),
            "frame":       spec.get("frame", ""),
            "coalesce":    ms_to_us(spec.get("coalesce_ms", 2), f"{ctx}.coalesce_ms"),
            "mirror":      spec.get("output_topic") or "/smartdata" + (topic if topic.startswith("/") else "/" + topic),
            "recovered":   (topic if topic.startswith("/") else "/" + topic) + "_recovered",
            "command":     (topic if topic.startswith("/") else "/" + topic) + "_command",
            "transducers": [],
        }
        topics.append(t)

        def add(name: str, unit: str, fields: list[str] | None, transform: Any,
                extra: dict | None = None) -> None:
            fwd, inv = transform_exprs(transform)
            td = {
                "name":    name,
                "cls":     pascal(name) + "_Transducer",
                "topic":   t,
                "unit":    unit,
                "unit_sym": unit_symbol(unit, f"{ctx}.unit"),
                "fields":  fields,
                "forward": fwd,
                "inverse": inv,
            }
            td.update(extra or {})
            t["transducers"].append(td)
            transducers.append(td)

        if kind == "scalar":
            add(t["name"], spec.get("unit"), [spec.get("value_field") or die(f"{ctx}: missing value_field")],
                spec.get("value_transform"))
        elif kind == "array":
            filt = spec.get("filter") or {"kind": "none"}
            if filt.get("kind", "none") not in ("none", "finite", "finite_within"):
                die(f"{ctx}: unknown filter.kind '{filt.get('kind')}'")
            add(t["name"], spec.get("unit"), None, spec.get("value_transform"),
                {"array_field": spec.get("array_field") or die(f"{ctx}: missing array_field"),
                 "filter": filt})
        elif kind == "composite":
            groups = spec.get("groups") or die(f"{ctx}: composite needs 'groups'")
            names: set[str] = set()
            for gi, g in enumerate(groups):
                gname = safe_ident(g.get("name", f"g{gi}"))
                if gname in names:
                    die(f"{ctx}: duplicate group name '{gname}'")
                names.add(gname)
                fields = g.get("fields") or die(f"{ctx}.groups[{gi}]: no fields")
                add(f"{t['name']}_{gname}", g.get("unit"), fields,
                    g.get("value_transform"))
            if sum(len(g["fields"]) for g in groups) > 64:
                die(f"{ctx}: composite messages support at most 64 fields")
        else:
            die(f"{ctx}: unknown decomposition '{kind}'")

    for k, td in enumerate(transducers):
        td["dev_base"] = k * DEV_RANGE
        td["devices"] = DEV_RANGE if td["fields"] is None else len(td["fields"])

    imports = build_imports(cfg.get("imports") or [], topics, transducers)
    return topics, transducers, imports


def device_id(value: Any, ctx: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < 0xFFFFFFFF:
        die(f"{ctx}: device must be an integer in [0, 0xFFFFFFFE] or 'any'")
    return value


def build_imports(specs: list[dict], topics: list[dict], transducers: list[dict]) -> list[dict]:
    """SmartData produced elsewhere (other gateways, SDAV/EPOS nodes) that the
    proxy subscribes to and publishes as ROS messages. They take no device
    block: their device ids are the producers'."""
    taken: set[str] = set()
    for t in topics:
        taken |= {t["topic"], t["mirror"]}
        taken.add(t["command"] if t["actuator"] else t["recovered"])

    imports: list[dict] = []
    for k, spec in enumerate(specs):
        topic = spec.get("topic") or die(f"imports[{k}]: entry without 'topic'")
        ctx = f"import[{topic}]"
        if topic in taken:
            die(f"{ctx}: topic already used by the gateway or another import")
        taken.add(topic)
        include, cpp = parse_type(spec.get("type") or die(f"{ctx}: missing 'type'"))
        kind = spec.get("decomposition")

        expiry = ms_to_us(spec.get("expiry_ms", 0), f"{ctx}.expiry_ms")
        period = None
        if "period_ms" in spec:
            period = ms_to_us(spec["period_ms"], f"{ctx}.period_ms")
            if period > 0 and 0 < expiry < period:
                die(f"{ctx}: expiry_ms smaller than period_ms is invalid")

        stamp = spec.get("stamp", "origin")
        if stamp not in ("origin", "receive"):
            die(f"{ctx}: stamp must be 'origin' or 'receive'")
        stamp_field = spec.get("stamp_field", "header.stamp")
        frame_id = spec.get("frame_id")
        if frame_id is not None and stamp_field is None:
            die(f"{ctx}: frame_id needs a message with a header")

        region = spec.get("region")
        if region is not None:
            region = {key: float(region.get(key, 0.0)) for key in ("x", "y", "z", "radius")}
            if region["radius"] < 0:
                die(f"{ctx}: region.radius must be non-negative")

        name = "import_" + safe_ident(topic)
        imp = {
            "topic":       topic,
            "name":        name,
            "include":     include,
            "cpp":         cpp,
            "kind":        kind,
            "expiry":      expiry,
            "period":      period,
            "stamp":       stamp,
            "stamp_field": stamp_field,
            "frame_id":    frame_id,
            "region":      region,
            "exclude_local": bool(spec.get("exclude_local", True)),
            "fields":      [],   # (cls, device or None = any, ROS field)
            "classes":     [],
        }

        def add_class(suffix: str, unit: Any, transform: Any, c: str) -> str:
            fwd, inv = transform_exprs(transform)
            cls = pascal(f"{name}_{suffix}") + "_Unit" if suffix else pascal(name) + "_Unit"
            imp["classes"].append({"cls": cls, "unit_sym": unit_symbol(unit, f"{c}.unit"),
                                   "forward": fwd, "inverse": inv})
            return cls

        def local_clash(unit_sym: str, dev: int, c: str) -> None:
            for td in transducers:
                if td["unit_sym"] == unit_sym and td["dev_base"] <= dev < td["dev_base"] + td["devices"]:
                    die(f"{c}: device {dev} belongs to the local transducer {td['cls']}")

        if kind == "scalar":
            cls = add_class("", spec.get("unit"), spec.get("value_transform"), ctx)
            dev = spec.get("device", "any")
            dev = None if dev == "any" else device_id(dev, f"{ctx}.device")
            if dev is not None:
                local_clash(imp["classes"][0]["unit_sym"], dev, ctx)
            field = spec.get("value_field") or die(f"{ctx}: missing value_field")
            imp["fields"].append((cls, dev, field))
        elif kind == "composite":
            if frame_id is not None and "{device}" in frame_id:
                die(f"{ctx}: '{{device}}' in frame_id only applies to scalar imports")
            groups = spec.get("groups") or die(f"{ctx}: composite needs 'groups'")
            seen_dev: set[tuple[str, int]] = set()
            for gi, g in enumerate(groups):
                gctx = f"{ctx}.groups[{gi}]"
                cls = add_class(safe_ident(g.get("name", f"g{gi}")), g.get("unit"),
                                g.get("value_transform"), gctx)
                unit_sym = imp["classes"][-1]["unit_sym"]
                for fi, f in enumerate(g.get("fields") or die(f"{gctx}: no fields")):
                    fctx = f"{gctx}.fields[{fi}]"
                    if not isinstance(f, dict) or "field" not in f or "device" not in f:
                        die(f"{fctx}: composite imports need {{field, device}} per field")
                    dev = device_id(f["device"], f"{fctx}.device")
                    if (unit_sym, dev) in seen_dev:
                        die(f"{fctx}: device {dev} listed twice for the same unit")
                    seen_dev.add((unit_sym, dev))
                    local_clash(unit_sym, dev, fctx)
                    imp["fields"].append((cls, dev, f["field"]))
            if len(imp["fields"]) > 64:
                die(f"{ctx}: composite messages support at most 64 fields")
        else:
            die(f"{ctx}: imports support scalar or composite decomposition")
        imports.append(imp)
    return imports


# ─── parsed_transducers.hpp ──────────────────────────────────────────────────


def emit_sensor_transducer(td: dict) -> str:
    t = td["topic"]
    if td["fields"] is not None:
        cases = "\n".join(
            f"        case {i}: raw = msg.{f}; break;" for i, f in enumerate(td["fields"]))
        body = f"""\
        double raw;
        switch (device() - DEV_BASE) {{
{cases}
        default: return;
        }}
        sample(to_si(raw), origin);"""
    else:
        af = td["array_field"]
        filt = td["filter"]
        checks = ""
        if filt.get("kind") in ("finite", "finite_within"):
            checks += "        if (!std::isfinite(raw)) return;  // invalid reading: keep the old, expiring value\n"
        if filt.get("kind") == "finite_within":
            checks += (f"        if (raw < msg.{filt['min_field']} || raw > msg.{filt['max_field']}) return;\n")
        body = f"""\
        const std::size_t i = device() - DEV_BASE;
        if (i >= msg.{af}.size()) return;
        const double raw = static_cast<double>(msg.{af}[i]);
{checks}        sample(to_si(raw), origin);"""

    return f"""\
// --- Automatically Generated Transducer: {td['name']} ---
// {t['topic']} ({t['cpp']}), unit {td['unit']}
class {td['cls']}
  : public sd::Transducer<{td['unit_sym']}>,
    public sd::Source<{t['cpp']}>::Observer
{{
public:
    using Message = {t['cpp']};

    static constexpr bool          active   = true;  // the ROS message is the interrupt
    static constexpr unsigned      TYPE     = SENSOR;
    static constexpr sd::Device_Id DEV_BASE = {td['dev_base']}u;
    static constexpr std::size_t   DEVICES  = {td['devices']}u;
    static constexpr const char *  TOPIC    = "{t['topic']}";

    {td['cls']}(sd::Device_Id dev, sd::Source<Message> & source)
      : Transducer(dev), source_(source) {{ source_.attach(this); }}

    ~{td['cls']}() override {{ source_.detach(this); }}

    static double to_si(double v)  {{ return {td['forward']}; }}
    static double to_ros(double v) {{ return {td['inverse']}; }}

    void update(const Message & msg, const sd::Origin & origin) override
    {{
{body}
    }}

private:
    sd::Source<Message> & source_;
}};
"""


def emit_actuator_transducer(td: dict) -> str:
    t = td["topic"]
    cases = "\n".join(
        f"            case {i}: m.{f} = static_cast<decltype(m.{f})>(raw); break;"
        for i, f in enumerate(td["fields"]))
    stamp = (f"            m.{t['stamp_field']} = sd::to_stamp(at.t);"
             if t["stamp_field"] else "            (void)at;")
    return f"""\
// --- Automatically Generated Transducer: {td['name']} ---
// {t['topic']} ({t['cpp']}), unit {td['unit']}
class {td['cls']} : public sd::Transducer<{td['unit_sym']}>
{{
public:
    using Message = {t['cpp']};

    static constexpr bool          active   = true;  // reports every new setpoint
    static constexpr unsigned      TYPE     = SENSOR | ACTUATOR;
    static constexpr sd::Device_Id DEV_BASE = {td['dev_base']}u;
    static constexpr std::size_t   DEVICES  = {td['devices']}u;
    static constexpr const char *  TOPIC    = "{t['topic']}";

    {td['cls']}(sd::Device_Id dev, sd::Sink<Message> & sink)
      : Transducer(dev), sink_(sink) {{}}

    static double to_si(double v)  {{ return {td['forward']}; }}
    static double to_ros(double v) {{ return {td['inverse']}; }}

    void actuate(double value) override
    {{
        const std::size_t i = device() - DEV_BASE;
        if (i >= DEVICES) return;
        const double raw = to_ros(value);
        const sd::Origin o = sink_.apply([&](Message & m, const sd::Origin & at) {{
            switch (i) {{
{cases}
            default: break;
            }}
{stamp}
        }});
        sample(value, o);
    }}

private:
    sd::Sink<Message> & sink_;
}};
"""


def emit_transducers(topics: list[dict], transducers: list[dict], src: str) -> str:
    includes = sorted({t["include"] for t in topics})
    sections = [emit_actuator_transducer(td) if td["topic"]["actuator"]
                else emit_sensor_transducer(td) for td in transducers]
    inc = "\n".join(f'#include "{i}"' for i in includes)
    body = "\n".join(sections)
    return f"""\
// AUTO-GENERATED by codegen/generate.py — do not edit.
// Source: {src}
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

{inc}

#include "si_units.hpp"
#include "smartdata/ros.hpp"
#include "smartdata/transducer.hpp"

namespace semantic_gateway::gen {{

{body}
}}  // namespace semantic_gateway::gen
"""


# ─── parsed_gateway.hpp ──────────────────────────────────────────────────────


def emit_gateway(topics: list[dict], transducers: list[dict], src: str) -> str:
    members_src: list[str] = []
    members_sd: list[str] = []
    members_sub: list[str] = []
    ctor_sinks: list[str] = []
    ctor_fixed: list[str] = []
    ctor_subs: list[str] = []
    helpers: list[str] = []
    counts: list[str] = []
    info: list[str] = []

    for t in topics:
        n = t["name"]
        if t["actuator"]:
            members_src.append(f"    std::unique_ptr<sd::Sink<{t['cpp']}>> sink_{n}_;")
            ctor_sinks.append(f"""\
        {{
            auto pub = node_->create_publisher<{t['cpp']}>("{t['topic']}", 10);
            sink_{n}_ = std::make_unique<sd::Sink<{t['cpp']}>>(
                [pub](const {t['cpp']} & m) {{ pub->publish(m); }},
                [this] {{ return locate("{t['frame']}"); }},
                std::chrono::microseconds({t['coalesce']}));
        }}""")
        else:
            members_src.append(f"    sd::Source<{t['cpp']}> src_{n}_;")
            members_sub.append(f"    rclcpp::Subscription<{t['cpp']}>::SharedPtr sub_{n}_;")
            grows = "".join(
                f"                grow_{td['name']}(m->{td['array_field']}.size());\n"
                for td in t["transducers"] if td["fields"] is None)
            ctor_subs.append(f"""\
        sub_{n}_ = node_->create_subscription<{t['cpp']}>(
            "{t['topic']}", rclcpp::SensorDataQoS(),
            [this]({t['cpp']}::ConstSharedPtr m) {{
                sd::Origin o = lookup_(m->{t['frame_field']}, m->{t['stamp_field']});
                o.t = sd::to_us(m->{t['stamp_field']});
{grows}                src_{n}_.publish(*m, o);
            }});""")

        for td in t["transducers"]:
            tn = td["name"]
            owner = f"*sink_{n}_" if t["actuator"] else f"src_{n}_"
            members_sd.append(
                f"    std::vector<std::unique_ptr<sd::Responsive_SmartData<{td['cls']}>>> sd_{tn}_;")
            helpers.append(f"""\
    void add_{tn}(std::size_t i)
    {{
        auto s = std::make_unique<sd::Responsive_SmartData<{td['cls']}>>(
            net_, {td['cls']}::DEV_BASE + static_cast<sd::Device_Id>(i),
            {t['expiry']} /* expiry µs */, {t['mode']}, {t['period']} /* period µs */, {owner});
        if (hook_) s->on_update([this](const sd::Snapshot & x) {{ hook_("{t['mirror']}", x); }});
        sd_{tn}_.push_back(std::move(s));
    }}""")
            if td["fields"] is None:
                helpers.append(f"""\
    void grow_{tn}(std::size_t n)
    {{
        if (n > {td['cls']}::DEVICES) n = {td['cls']}::DEVICES;
        while (sd_{tn}_.size() < n) add_{tn}(sd_{tn}_.size());
    }}""")
            else:
                ctor_fixed.append(
                    f"        for (std::size_t i = 0; i < {td['cls']}::DEVICES; ++i) add_{tn}(i);")
            counts.append(f"sd_{tn}_.size()")
            info.append(
                f'    {{"{t["topic"]}", "{td["cls"]}", {td["unit_sym"]}, '
                f'{td["dev_base"]}u, {td["devices"]}u, {"true" if t["actuator"] else "false"}}},')

    mirror_topics = ", ".join(f'"{t["mirror"]}"' for t in topics)
    nl = "\n"
    return f"""\
// AUTO-GENERATED by codegen/generate.py — do not edit.
// Source: {src}
#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "smartdata/smartdata.hpp"
#include "parsed_transducers.hpp"

namespace semantic_gateway::gen {{

// Resolves where a frame is (in the gateway's reference frame) at `stamp`.
using Frame_Lookup = std::function<sd::Origin(const std::string & frame,
                                              const builtin_interfaces::msg::Time & stamp)>;

// Called on every SmartData update (local instrumentation, e.g. ROS mirror).
using Update_Hook = std::function<void(const std::string & mirror_topic,
                                       const sd::Snapshot & snapshot)>;

struct Transducer_Info {{
    const char *  topic;
    const char *  transducer;
    uint32_t      unit;
    sd::Device_Id dev_base;
    std::size_t   devices;  // maximum for vector transducers
    bool          actuator;
}};

inline constexpr Transducer_Info TRANSDUCERS[] = {{
{nl.join(info)}
}};

inline constexpr const char * MIRROR_TOPICS[] = {{ {mirror_topics} }};

// Local side of the gateway: every transducer instance exported as a
// Responsive SmartData — ADVERTISED sensors, COMMANDED actuators.
class Gateway {{
public:
    Gateway(rclcpp::Node * node, sd::Network & net, Frame_Lookup lookup,
            Update_Hook hook = {{}})
      : node_(node), net_(net), lookup_(std::move(lookup)), hook_(std::move(hook))
    {{
{nl.join(ctor_sinks)}
{nl.join(ctor_fixed)}
        // vector transducers are instantiated when the first message tells
        // how many elements the sensor has
{nl.join(ctor_subs)}
    }}

    Gateway(const Gateway &) = delete;
    Gateway & operator=(const Gateway &) = delete;

    std::size_t smartdata() const {{ return {" + ".join(counts) if counts else "0"}; }}

private:
    sd::Origin locate(const std::string & frame)
    {{
        const rclcpp::Time now = node_->now();
        builtin_interfaces::msg::Time stamp = now;
        sd::Origin o = lookup_(frame, stamp);
        o.t = now.nanoseconds() / 1000;
        return o;
    }}

{nl.join(helpers)}

    rclcpp::Node * node_;
    sd::Network &  net_;
    Frame_Lookup   lookup_;
    Update_Hook    hook_;

    // sources/sinks outlive the transducers attached to them
{nl.join(members_src)}

{nl.join(members_sd)}

    // subscriptions go first on destruction: no data during teardown
{nl.join(members_sub)}
}};

}}  // namespace semantic_gateway::gen
"""


# ─── parsed_proxies.hpp ──────────────────────────────────────────────────────


def emit_composite_feed(n: str, cpp: str, entries: list[tuple[int, str, str]],
                        start: list[str], why: str) -> tuple[str, str, str]:
    """(feed + take, state, drain) rebuilding one message from several SmartData.

    entries: (device id, ROS field, class with UNIT/to_ros) per field.
    start:   statements run on `s` when a new sample (origin time) begins.
    Responses that share an origin time are one sample; the message goes out
    once every field arrived, when a newer sample begins, or on drain()."""
    ids = []
    sets = []
    for bit, (dev, f, cls) in enumerate(entries):
        ids.append(f"        case {dev}u: field = {bit}; break;")
        sets.append(
            f"            case {bit}: s.msg.{f} = "
            f"static_cast<decltype(s.msg.{f})>({cls}::to_ros(v)); break;")
    bits = len(entries)
    full = "~0ull" if bits == 64 else f"((1ull << {bits}) - 1)"
    id_block = "\n".join(ids)
    set_block = "\n".join(sets)
    start_block = "\n".join(f"                {line}" for line in start)
    why_block = "\n".join(f"        // {line}" for line in why.split("\n"))
    feed = f"""\
    void on_{n}(sd::Device_Id dev, double v, sd::Time t)
    {{
{why_block}
        int field = -1;
        switch (dev) {{
{id_block}
        default: return;
        }}
        std::optional<{cpp}> out, done;
        {{
            std::lock_guard<std::mutex> lk(mutex_);
            auto & s = st_{n}_;
            if (t <= s.last) return;
            if (s.has && s.t != t) out = take_{n}(s);  // incomplete sample: publish what arrived
            if (!s.has) {{
                s.has = true;
                s.t   = t;
{start_block}
            }}
            switch (field) {{
{set_block}
            default: break;
            }}
            s.seen |= (1ull << field);
            s.touched = true;
            if (s.seen == {full}) done = take_{n}(s);
        }}
        if (out)  {{ pub_{n}_->publish(*out);  ++published_; }}
        if (done) {{ pub_{n}_->publish(*done); ++published_; }}
    }}

    {cpp} take_{n}(State_{n} & s)
    {{
        {cpp} m = std::move(s.msg);
        s.last = s.t;
        s.has  = false;
        s.seen = 0;
        s.msg  = {cpp}{{}};
        return m;
    }}"""
    state = f"""\
    struct State_{n} {{ bool has{{false}}; bool touched{{false}}; sd::Time t{{0}}; sd::Time last{{-1}}; uint64_t seen{{0}}; {cpp} msg{{}}; }};
    State_{n} st_{n}_;"""
    drain = f"""\
        {{
            std::optional<{cpp}> out;
            {{
                std::lock_guard<std::mutex> lk(mutex_);
                auto & s = st_{n}_;
                if (s.has && !s.touched) out = take_{n}(s);
                s.touched = false;
            }}
            if (out) {{ pub_{n}_->publish(*out); ++published_; }}
        }}"""
    return feed, state, drain


def emit_imports(imports: list[dict], transducers: list[dict]) -> dict[str, list[str]]:
    """Pieces of class Proxies for the imported SmartData (see build_imports)."""
    out: dict[str, list[str]] = {k: [] for k in
                                 ("units", "pubs", "ctor_pubs", "ctor_px", "members", "feeds", "drains")}
    for imp in imports:
        n, cpp = imp["name"], imp["cpp"]
        sf = imp["stamp_field"]
        for c in imp["classes"]:
            out["units"].append(f"""\
// --- Imported SmartData: {imp['topic']} ({cpp}) ---
struct {c['cls']} {{
    static constexpr uint32_t UNIT = {c['unit_sym']};
    static double to_ros(double v) {{ return {c['inverse']}; }}
}};""")

        out["pubs"].append(f"    rclcpp::Publisher<{cpp}>::SharedPtr pub_{n}_;")
        out["ctor_pubs"].append(
            f'        pub_{n}_ = node->create_publisher<{cpp}>("{imp["topic"]}", rclcpp::SensorDataQoS());')

        r = imp["region"]
        region = (f"make_region({r['x']!r}, {r['y']!r}, {r['z']!r}, {r['radius']!r})"
                  if r else "region")
        period = "period" if imp["period"] is None else f"{imp['period']}"
        stamp = "node_->now()" if imp["stamp"] == "receive" else "sd::to_stamp(t)"
        frame_root = sf.rsplit(".", 1)[0] + ".frame_id" if sf and "." in sf else "frame_id"

        for k, (cls, dev, _field) in enumerate(imp["fields"]):
            px = f"sd::Interested_SmartData<{cls}::UNIT>"
            member = f"px_{n}_{k}_"
            out["members"].append(f"    std::unique_ptr<{px}> {member};")
            out["ctor_px"].append(f"""\
        {member} = std::make_unique<{px}>(
            net, {region}, client_expiry({imp['expiry']}, {period}), {period}, sd::ALL,
            {'sd::ANY_DEVICE' if dev is None else f'{dev}u'}, {px}::Fuser{{}},
            [this](const {px}::Sample & s) {{ on_{n}(s.device, s.value, s.origin.t); }});""")

        if imp["kind"] == "scalar":
            cls, dev, field = imp["fields"][0]
            lines = []
            if dev is None and imp["exclude_local"]:
                lines.append(f"        if (local_device<{cls}::UNIT>(dev)) return;  // ours, not imported")
            lines.append(f"""\
        {{
            std::lock_guard<std::mutex> lk(mutex_);
            auto it = last_{n}_.find(dev);
            if (it != last_{n}_.end() && t <= it->second) return;  // already published
            last_{n}_[dev] = t;
        }}
        {cpp} m;""")
            if sf:
                lines.append(f"        m.{sf} = {stamp};")
            if imp["frame_id"] is not None:
                fid = imp["frame_id"]
                if "{device}" in fid:
                    pre, post = fid.split("{device}", 1)
                    expr = f"std::string({cstr(pre)}) + std::to_string(dev)"
                    if post:
                        expr += f" + {cstr(post)}"
                else:
                    expr = cstr(fid)
                lines.append(f"        m.{frame_root} = {expr};")
            lines.append(f"""\
        m.{field} = static_cast<decltype(m.{field})>({cls}::to_ros(v));
        pub_{n}_->publish(m);
        ++published_;""")
            body = "\n".join(lines)
            out["feeds"].append(f"""\

    std::unordered_map<sd::Device_Id, sd::Time> last_{n}_;  // per producer

    void on_{n}(sd::Device_Id dev, double v, sd::Time t)
    {{
{body}
    }}""")
        else:
            start = []
            if sf:
                start.append(f"s.msg.{sf} = {stamp};")
            if imp["frame_id"] is not None:
                start.append(f"s.msg.{frame_root} = {cstr(imp['frame_id'])};")
            feed, state, drain = emit_composite_feed(
                n, cpp, [(dev, f, cls) for cls, dev, f in imp["fields"]], start,
                "only the devices this import lists; another producer of the same\n"
                "unit is not part of this message")
            out["feeds"].append("\n" + state + "\n\n" + feed)
            out["drains"].append(drain)
    return out


def emit_recompose(t: dict) -> tuple[str, str, str]:
    """(state struct + feed function, drain block, publisher member) for a sensor topic."""
    n = t["name"]
    cpp = t["cpp"]
    stamp = t["stamp_field"]
    tds = t["transducers"]
    kind = t["kind"]

    if kind == "scalar":
        td = tds[0]
        f = td["fields"][0]
        feed = f"""\
    void on_{n}(sd::Device_Id dev, double v, sd::Time t)
    {{
        if (dev != {td['cls']}::DEV_BASE) return;
        {cpp} m;
        {{
            std::lock_guard<std::mutex> lk(mutex_);
            if (t <= st_{n}_.last) return;  // already published
            st_{n}_.last = t;
        }}
        m.{stamp} = sd::to_stamp(t);
        m.{f} = static_cast<decltype(m.{f})>({td['cls']}::to_ros(v));
        pub_{n}_->publish(m);
        ++published_;
    }}"""
        state = f"""\
    struct State_{n} {{ sd::Time last{{-1}}; }};
    State_{n} st_{n}_;"""
        drain = ""
    elif kind == "array":
        td = tds[0]
        af = td["array_field"]
        feed = f"""\
    void on_{n}(sd::Device_Id dev, double v, sd::Time t)
    {{
        if (dev < {td['cls']}::DEV_BASE || dev - {td['cls']}::DEV_BASE >= {td['cls']}::DEVICES) return;
        std::optional<{cpp}> out;
        {{
            std::lock_guard<std::mutex> lk(mutex_);
            auto & s = st_{n}_;
            if (t <= s.last) return;
            if (s.has && s.t != t) out = take_{n}(s);  // a new scan began
            if (!s.has) {{
                s.has = true;
                s.t   = t;
                s.msg.{stamp} = sd::to_stamp(t);
            }}
            const std::size_t i = dev - {td['cls']}::DEV_BASE;
            if (i >= s.msg.{af}.size()) s.msg.{af}.resize(i + 1, 0);
            s.msg.{af}[i] = static_cast<typename decltype(s.msg.{af})::value_type>(
                {td['cls']}::to_ros(v));
            s.touched = true;
        }}
        if (out) {{ pub_{n}_->publish(*out); ++published_; }}
    }}

    {cpp} take_{n}(State_{n} & s)
    {{
        {cpp} m = std::move(s.msg);
        s.last = s.t;
        s.has  = false;
        s.msg  = {cpp}{{}};
        return m;
    }}"""
        state = f"""\
    struct State_{n} {{ bool has{{false}}; bool touched{{false}}; sd::Time t{{0}}; sd::Time last{{-1}}; {cpp} msg{{}}; }};
    State_{n} st_{n}_;"""
        drain = f"""\
        {{
            std::optional<{cpp}> out;
            {{
                std::lock_guard<std::mutex> lk(mutex_);
                auto & s = st_{n}_;
                if (s.has && !s.touched) out = take_{n}(s);
                s.touched = false;
            }}
            if (out) {{ pub_{n}_->publish(*out); ++published_; }}
        }}"""
    else:  # composite
        entries = [(td["dev_base"] + i, f, td["cls"])
                   for td in tds for i, f in enumerate(td["fields"])]
        feed, state, drain = emit_composite_feed(
            n, cpp, entries, [f"s.msg.{stamp} = sd::to_stamp(t);"],
            "other topics' transducers of the same unit reach this proxy too\n"
            "(ANY_DEVICE): drop them before they touch the sample being built")

    return state + "\n\n" + feed, drain, f"    rclcpp::Publisher<{cpp}>::SharedPtr pub_{n}_;"


def emit_proxies(topics: list[dict], transducers: list[dict], imports: list[dict],
                 src: str) -> str:
    recompose: list[str] = []
    drains: list[str] = []
    pubs: list[str] = []
    proxies_m: list[str] = []
    cmd_subs_m: list[str] = []
    ctor_pubs: list[str] = []
    ctor_px: list[str] = []
    ctor_cmd: list[str] = []

    for t in topics:
        n = t["name"]
        if not t["actuator"]:
            r, d, p = emit_recompose(t)
            recompose.append(r)
            if d:
                drains.append(d)
            pubs.append(p)
            ctor_pubs.append(
                f'        pub_{n}_ = node->create_publisher<{t["cpp"]}>("{t["recovered"]}", rclcpp::SensorDataQoS());')
            for td in t["transducers"]:
                tn = td["name"]
                px = f"sd::Interested_SmartData<{td['cls']}::UNIT>"
                proxies_m.append(f"    std::unique_ptr<{px}> px_{tn}_;")
                ctor_px.append(f"""\
        px_{tn}_ = std::make_unique<{px}>(
            net, region, client_expiry({t['expiry']}, period), period, sd::ALL, sd::ANY_DEVICE,
            {px}::Fuser{{}},
            [this](const {px}::Sample & s) {{ on_{n}(s.device, s.value, s.origin.t); }});""")
        else:
            assigns: list[str] = []
            for td in t["transducers"]:
                tn = td["name"]
                px = f"sd::Interested_SmartData<{td['cls']}::UNIT>"
                proxies_m.append(f"    std::vector<std::unique_ptr<{px}>> cmd_{tn}_;")
                ctor_px.append(f"""\
        for (std::size_t i = 0; i < {td['cls']}::DEVICES; ++i) {{
            cmd_{tn}_.push_back(std::make_unique<{px}>(
                net, region, {t['expiry']}, 0, sd::SINGLE,
                {td['cls']}::DEV_BASE + static_cast<sd::Device_Id>(i)));
        }}""")
                for i, f in enumerate(td["fields"]):
                    assigns.append(f"                *cmd_{tn}_[{i}] = {td['cls']}::to_si(m->{f});")
            cmd_subs_m.append(f"    rclcpp::Subscription<{t['cpp']}>::SharedPtr sub_{n}_;")
            body = "\n".join(assigns)
            ctor_cmd.append(f"""\
        sub_{n}_ = node->create_subscription<{t['cpp']}>(
            "{t['command']}", 10,
            [this]({t['cpp']}::ConstSharedPtr m) {{
{body}
                ++commands_;
            }});""")

    imp = emit_imports(imports, transducers)
    recompose += imp["feeds"]
    drains += imp["drains"]
    pubs += imp["pubs"]
    ctor_pubs += imp["ctor_pubs"]
    ctor_px += imp["ctor_px"]
    proxies_m += imp["members"]
    local = " ||\n               ".join(
        f"(U == {td['cls']}::UNIT && dev - {td['cls']}::DEV_BASE < {td['cls']}::DEVICES)"
        for td in transducers) or "((void)dev, false)"
    units = "\n\n".join(imp["units"])

    includes = sorted({t["include"] for t in topics} | {i["include"] for i in imports})
    inc = "\n".join(f'#include "{i}"' for i in includes)
    nl = "\n"
    return f"""\
// AUTO-GENERATED by codegen/generate.py — do not edit.
// Source: {src}
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

{inc}

#include "smartdata/ros.hpp"
#include "smartdata/smartdata.hpp"
#include "parsed_transducers.hpp"

namespace semantic_gateway::gen {{

{units}

// Remote side of the gateway. Sensor data is obtained through
// Interested SmartData (one interest per transducer class, every device),
// and the ROS messages are rebuilt by grouping the responses that share an
// origin time. Writes to actuators go out as SmartData commands. Imported
// SmartData (produced by any other node) are published on their own topics.
class Proxies {{
public:
    // region: where the interests apply; period: 0 for event-driven
    // interests, > 0 for time-triggered ones (µs). Imports may override both.
    Proxies(rclcpp::Node * node, sd::Network & net, const sd::Region & region,
            sd::Microsecond period = 0)
      : node_(node)
    {{
{nl.join(ctor_pubs)}
{nl.join(ctor_px)}
{nl.join(ctor_cmd)}
    }}

    Proxies(const Proxies &) = delete;
    Proxies & operator=(const Proxies &) = delete;

    // Publishes vector/composite samples that stopped receiving data since
    // the previous call (call it periodically).
    void drain()
    {{
{nl.join(drains)}
    }}

    std::size_t published() const {{ return published_.load(); }}
    std::size_t commands()  const {{ return commands_.load(); }}

private:
    // The proxy decides how long data stays usable; it may not be shorter
    // than the period it asks for.
    static sd::Microsecond client_expiry(sd::Microsecond expiry, sd::Microsecond period)
    {{
        return (expiry > 0 && period > expiry) ? period : expiry;
    }}

    static sd::Region make_region(double x, double y, double z, double radius)
    {{
        sd::Region r;
        r.x = x;
        r.y = y;
        r.z = z;
        r.radius = radius;
        return r;
    }}

    // True for the devices of this package's own transducers of unit U.
    template<uint32_t U>
    static bool local_device(sd::Device_Id dev)
    {{
        return {local};
    }}

    rclcpp::Node * node_;

{nl.join(recompose)}

    std::mutex mutex_;
    std::atomic<std::size_t> published_{{0}};
    std::atomic<std::size_t> commands_{{0}};

{nl.join(pubs)}

{nl.join(proxies_m)}

{nl.join(cmd_subs_m)}
}};

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
        cfg = yaml.safe_load(f) or {}

    topics, transducers, imports = build_model(cfg)
    os.makedirs(args.out, exist_ok=True)

    outputs = {
        "parsed_transducers.hpp": emit_transducers(topics, transducers, args.yaml),
        "parsed_gateway.hpp":     emit_gateway(topics, transducers, args.yaml),
        "parsed_proxies.hpp":     emit_proxies(topics, transducers, imports, args.yaml),
    }
    for fname, content in outputs.items():
        with open(os.path.join(args.out, fname), "w") as f:
            f.write(content)

    print(f"codegen: wrote {len(outputs)} headers to {args.out} "
          f"({len(topics)} topics, {len(transducers)} transducer classes, "
          f"{len(imports)} imports)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
