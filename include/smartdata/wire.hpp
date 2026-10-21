#pragma once

// wire.hpp — SmartData network messages (TSTP-style encoding).
//
// The paper leaves the packet encoding to the communication protocol ("the
// actual encoding of SmartData attributes in a network packet is
// protocol-dependent", Fröhlich 2018 §1). It follows the TSTP message layout
// used by LISHA's SmartData implementation (SDAV, smartdata.h
// _Header/Interest/Response/Command) at the 32-bit centimetre scale, so the
// byte offsets of every field match that implementation:
//
//   Header   (60 B)  config | mode | misc | loc.conf | origin(x,y,z,t) |
//                    unit | device | signature | last_hop(x,y,z,t)
//   Interest (104 B) Header | region(x,y,z,r,t0,t1) | expiry | period
//   Response (70 B + value)  Header | expiry | fragment | value
//   Command  (84 B + value)  Header | radius | t1 | expiry | period | value
//
// Values are IEEE-754 binary64 (unit NUM field = Float64). All fields are in
// host byte order, which is little-endian on every platform we target.
// See docs/wire_format.md for the field-by-field rationale.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <type_traits>

namespace sd {

using Device_Id   = uint32_t;
using Time        = int64_t;   // µs since the epoch of the runtime clock
using Microsecond = int64_t;   // relative durations (expiry, period)

constexpr Time      INFINITE   = std::numeric_limits<Time>::max();
constexpr Device_Id ANY_DEVICE = 0xFFFFFFFFu;  // Interest/Command wildcard

// Message type (bits 4..3 of Header::config)
enum Type : uint8_t {
    INTEREST = 0,
    RESPONSE = 1,
    COMMAND  = 2,
    CONTROL  = 3,
};

// Header::mode
//   bits 1..0  mode       (PRIVATE/ADVERTISED/COMMANDED or SINGLE/ALL)
//   bits 3..2  operation  (ADVERTISE/CONCEAL/RESPOND or ANNOUNCE/REVOKE)
//   bits 7..4  subtype
enum : uint8_t {
    MODE_MASK      = 0x3 << 0,
    OPERATION_MASK = 0x3 << 2,
    SUBTYPE_MASK   = 0xf << 4,

    // Interested modes and operations
    SINGLE   = 0 << 0,  // one response per interest is enough
    ALL      = 1 << 0,  // every matching transducer should respond
    ANNOUNCE = 0 << 2,
    REVOKE   = 1 << 2,

    // Responsive modes and operations
    PRIVATE    = 0 << 0,  // local access only
    ADVERTISED = 1 << 0,  // can bind to interests (remotely monitored)
    COMMANDED  = 3 << 0,  // advertised and remotely controlled
    ADVERTISE  = 0 << 2,
    CONCEAL    = 1 << 2,
    RESPOND    = 2 << 2,
};

constexpr uint8_t SCALE_CM_32 = 3;  // 32-bit coordinates in centimetres
constexpr uint8_t VERSION     = 4;  // TSTP "V1/mobile" — the robot moves

namespace wire {

#pragma pack(push, 1)

struct Space {
    int32_t x, y, z;  // cm
};

struct Spacetime {
    Space   space;
    int64_t t;        // µs
};

struct Header {
    uint8_t   config;               // scale[7:6] | time_req[5] | type[4:3] | version[2:0]
    uint8_t   mode;
    uint8_t   misc;                 // uncertainty (responses) / unused
    uint8_t   location_confidence;  // 0..100 %
    Spacetime origin;
    uint32_t  unit;
    uint32_t  device;
    uint64_t  signature;
    Spacetime last_hop;
};

struct Region {
    Space    center;
    uint32_t radius;  // cm
    int64_t  t0;
    int64_t  t1;
};

struct Interest {
    Header  header;
    Region  region;
    int64_t expiry;
    int32_t period;   // µs; 0 = event-driven
};

struct Response {
    Header   header;
    int64_t  expiry;
    uint16_t fragment;
    double   value;   // absent in ADVERTISE/CONCEAL (70-byte message)
};

struct Command {
    Header   header;  // origin = region centre and t0
    uint32_t radius;
    int64_t  t1;
    int64_t  expiry;
    int32_t  period;
    double   value;
};

#pragma pack(pop)

static_assert(sizeof(Header)   ==  60, "TSTP header (CM_32) is 60 bytes");
static_assert(sizeof(Region)   ==  32, "region is 32 bytes");
static_assert(sizeof(Interest) == 104, "interest is 104 bytes");
static_assert(sizeof(Response) ==  78, "response with Float64 value is 78 bytes");
static_assert(sizeof(Command)  ==  92, "command with Float64 value is 92 bytes");
static_assert(std::is_trivially_copyable<Response>::value, "wire types must be POD");

constexpr std::size_t RESPONSE_NO_VALUE = offsetof(Response, value);  // 70

inline uint8_t config(Type t) noexcept
{
    return static_cast<uint8_t>((SCALE_CM_32 & 0x3) << 6 | 0 << 5 |
                                (t & 0x3) << 3 | (VERSION & 0x7));
}

inline Type type_of(const Header & h) noexcept
{
    return static_cast<Type>((h.config >> 3) & 0x3);
}

// Copies a message out of a datagram. Returns nullopt when it is too short.
template<typename M>
std::optional<M> parse(const uint8_t * data, std::size_t len, std::size_t min = sizeof(M))
{
    if (len < min) return std::nullopt;
    M m{};
    std::memcpy(&m, data, len < sizeof(M) ? len : sizeof(M));
    return m;
}

}  // namespace wire

// ── Runtime-side representations (metres, doubles) ──────────────────────────

inline int32_t to_cm(double metres) noexcept
{
    const double cm = std::round(metres * 100.0);
    constexpr double lo = std::numeric_limits<int32_t>::min();
    constexpr double hi = std::numeric_limits<int32_t>::max();
    if (!(cm >= lo)) return std::numeric_limits<int32_t>::min();
    if (cm > hi)     return std::numeric_limits<int32_t>::max();
    return static_cast<int32_t>(cm);
}

inline double from_cm(int32_t cm) noexcept { return cm / 100.0; }

// Where and when a piece of data was produced (Origin(x, y, z, t) in the
// paper). Coordinates are metres in the gateway's reference frame.
struct Origin {
    double  x{0.0}, y{0.0}, z{0.0};
    Time    t{0};
    uint8_t confidence{0};  // location confidence, 0..100 %

    wire::Spacetime to_wire() const noexcept
    {
        return {{to_cm(x), to_cm(y), to_cm(z)}, t};
    }

    static Origin from_wire(const wire::Spacetime & st, uint8_t conf) noexcept
    {
        return {from_cm(st.space.x), from_cm(st.space.y), from_cm(st.space.z),
                st.t, conf};
    }
};

// Space-Time Region(x, y, z, r, t0, tf) used by interests and commands.
struct Region {
    double x{0.0}, y{0.0}, z{0.0};
    double radius{0.0};  // metres
    Time   t0{0};
    Time   t1{INFINITE};

    static Region everywhere() noexcept
    {
        Region r;
        r.radius = std::numeric_limits<uint32_t>::max() / 100.0;
        return r;
    }

    bool contains_space(double px, double py, double pz) const noexcept
    {
        const double dx = px - x, dy = py - y, dz = pz - z;
        return std::sqrt(dx * dx + dy * dy + dz * dz) <= radius;
    }

    bool contains_time(Time t) const noexcept { return t >= t0 && t <= t1; }

    bool contains(const Origin & o) const noexcept
    {
        return contains_time(o.t) && contains_space(o.x, o.y, o.z);
    }

    wire::Region to_wire() const noexcept
    {
        const double r_cm = std::round(radius * 100.0);
        const uint32_t r = r_cm >= std::numeric_limits<uint32_t>::max()
            ? std::numeric_limits<uint32_t>::max()
            : static_cast<uint32_t>(r_cm < 0 ? 0 : r_cm);
        return {{to_cm(x), to_cm(y), to_cm(z)}, r, t0, t1};
    }

    static Region from_wire(const wire::Region & w) noexcept
    {
        Region r;
        r.x = from_cm(w.center.x);
        r.y = from_cm(w.center.y);
        r.z = from_cm(w.center.z);
        r.radius = w.radius / 100.0;
        r.t0 = w.t0;
        r.t1 = w.t1;
        return r;
    }

    bool operator==(const Region & o) const noexcept
    {
        const auto a = to_wire(), b = o.to_wire();
        return std::memcmp(&a, &b, sizeof(a)) == 0;
    }
};

// ── Message builders ───────────────────────────────────────────────────────

inline wire::Header make_header(Type type, uint8_t mode, uint8_t misc,
                                const wire::Spacetime & origin, uint8_t confidence,
                                uint32_t unit, Device_Id device) noexcept
{
    wire::Header h{};
    h.config              = wire::config(type);
    h.mode                = mode;
    h.misc                = misc;
    h.location_confidence = confidence;
    h.origin              = origin;
    h.unit                = unit;
    h.device              = device;
    h.signature           = 0;       // no security layer (out of scope)
    h.last_hop            = origin;  // single-hop multicast segment
    return h;
}

}  // namespace sd
