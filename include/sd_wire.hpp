#pragma once

// sd_wire.hpp — packed 72-byte binary envelope for the SmartData bus.
//
// See docs/wire_format.md for the layout rationale and field semantics.
// The struct mirrors the IDL one-to-one; conversion is a field copy.

#include <cstdint>
#include <cstring>
#include <type_traits>

#include "semantic_gateway/msg/smart_data.hpp"

namespace semantic_gateway {

#pragma pack(push, 1)
struct SDWire {
    uint32_t unit;
    double   value;
    double   origin_x;
    double   origin_y;
    double   origin_z;
    uint32_t origin_device;
    uint64_t origin_time;
    uint64_t expiry;
    uint8_t  mac[16];
};
#pragma pack(pop)

static_assert(sizeof(SDWire) == 72,
    "SDWire must be exactly 72 bytes — wire format is fixed");
static_assert(std::is_trivially_copyable<SDWire>::value,
    "SDWire must be trivially copyable to memcpy into UDP payload");

constexpr std::size_t SD_WIRE_SIZE = sizeof(SDWire);

inline SDWire to_wire(const msg::SmartData & sd) noexcept
{
    SDWire w{};
    w.unit          = sd.unit;
    w.value         = sd.value;
    w.origin_x      = sd.origin_x;
    w.origin_y      = sd.origin_y;
    w.origin_z      = sd.origin_z;
    w.origin_device = sd.origin_device;
    w.origin_time   = sd.origin_time;
    w.expiry        = sd.expiry;
    std::memcpy(w.mac, sd.mac.data(), 16);
    return w;
}

inline msg::SmartData from_wire(const SDWire & w) noexcept
{
    msg::SmartData sd;
    sd.unit          = w.unit;
    sd.value         = w.value;
    sd.origin_x      = w.origin_x;
    sd.origin_y      = w.origin_y;
    sd.origin_z      = w.origin_z;
    sd.origin_device = w.origin_device;
    sd.origin_time   = w.origin_time;
    sd.expiry        = w.expiry;
    std::memcpy(sd.mac.data(), w.mac, 16);
    return sd;
}

}  // namespace semantic_gateway
