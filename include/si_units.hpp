#pragma once

#include <cstdint>
#include <string>

// si_units.hpp — 32-bit dimensional SI unit identifier 
//
// Layout:
//
//  bit  31    30..29  28..27  26..24  23..21  20..18  17..15  14..12  11..9   8..6    5..3    2..0
//       T    N       M       sr      rad     m       kg      s       A       K       mol     cd
//
//  T : 1 bit  — 0 = digital, 1 = SI physical quantity
//  N : 2 bits — numeric type     {0: Int32, 1: Int64, 2: Float32, 3: Float64}
//  M : 2 bits — modifier         {0: direct, 1: 1/U, 2: log(U), 3: 1/log(U)}
//
//  each of the 9 dimension fields stores an exponent with offset +4
//  (encoded value 4 <-> exponent 0; representable range −4 … +3).
//

namespace si_units {

// bit offsets
constexpr unsigned BIT_T   = 31;
constexpr unsigned BIT_N   = 29;
constexpr unsigned BIT_M   = 27;
constexpr unsigned BIT_SR  = 24;
constexpr unsigned BIT_RAD = 21;
constexpr unsigned BIT_M_  = 18;  // length (meter)
constexpr unsigned BIT_KG  = 15;
constexpr unsigned BIT_S   = 12;
constexpr unsigned BIT_A   =  9;
constexpr unsigned BIT_K   =  6;
constexpr unsigned BIT_MOL =  3;
constexpr unsigned BIT_CD  =  0;

constexpr int EXP_OFFSET = 4;     // exponent 0 stored as 4

// type / numeric / modifier enums
enum class Type     : uint32_t { Digital = 0, SI = 1 };
enum class Numeric  : uint32_t { Int32 = 0, Int64 = 1, Float32 = 2, Float64 = 3 };
enum class Modifier : uint32_t { Direct = 0, Inverse = 1, Log = 2, InvLog = 3 };

// encoder
// each dimension exponent is given in its natural (signed) form
constexpr uint32_t encode_si(
    Numeric n, Modifier m,
    int sr, int rad, int m_len,
    int kg, int s,   int A,
    int K,  int mol, int cd) noexcept
{
    const auto enc = [](int e) -> uint32_t {
        return static_cast<uint32_t>(e + EXP_OFFSET) & 0x7u;
    };
    return (static_cast<uint32_t>(Type::SI)    << BIT_T)
         | (static_cast<uint32_t>(n)           << BIT_N)
         | (static_cast<uint32_t>(m)           << BIT_M)
         | (enc(sr)    << BIT_SR)
         | (enc(rad)   << BIT_RAD)
         | (enc(m_len) << BIT_M_)
         | (enc(kg)    << BIT_KG)
         | (enc(s)     << BIT_S)
         | (enc(A)     << BIT_A)
         | (enc(K)     << BIT_K)
         | (enc(mol)   << BIT_MOL)
         | (enc(cd)    << BIT_CD);
}

// convenience for ROS gateway, every value is Float64 with no modifier
constexpr uint32_t si(
    int sr, int rad, int m_len, int kg, int s,
    int A,  int K,   int mol,   int cd) noexcept
{
    return encode_si(Numeric::Float64, Modifier::Direct,
                     sr, rad, m_len, kg, s, A, K, mol, cd);
}

// pre-computed SI quantitier used by the gateway
//                                        sr rad  m  kg  s  A  K mol cd
constexpr uint32_t METER              = si(0, 0,  1, 0,  0, 0, 0, 0, 0);
constexpr uint32_t KILOGRAM           = si(0, 0,  0, 1,  0, 0, 0, 0, 0);
constexpr uint32_t SECOND             = si(0, 0,  0, 0,  1, 0, 0, 0, 0);
constexpr uint32_t AMPERE             = si(0, 0,  0, 0,  0, 1, 0, 0, 0);
constexpr uint32_t KELVIN             = si(0, 0,  0, 0,  0, 0, 1, 0, 0);
constexpr uint32_t MOLE               = si(0, 0,  0, 0,  0, 0, 0, 1, 0);
constexpr uint32_t CANDELA            = si(0, 0,  0, 0,  0, 0, 0, 0, 1);
constexpr uint32_t RADIAN             = si(0, 1,  0, 0,  0, 0, 0, 0, 0);
constexpr uint32_t METER_PER_SECOND   = si(0, 0,  1, 0, -1, 0, 0, 0, 0);
constexpr uint32_t METER_PER_SEC_SQ   = si(0, 0,  1, 0, -2, 0, 0, 0, 0);
constexpr uint32_t RADIAN_PER_SECOND  = si(0, 1,  0, 0, -1, 0, 0, 0, 0);
constexpr uint32_t NEWTON             = si(0, 0,  1, 1, -2, 0, 0, 0, 0);
constexpr uint32_t PASCAL             = si(0, 0, -1, 1, -2, 0, 0, 0, 0);
constexpr uint32_t VOLT               = si(0, 0,  2, 1, -3,-1, 0, 0, 0);

// decoder
struct Decoded {
    Type     type;
    Numeric  numeric;
    Modifier modifier;
    int sr, rad, m, kg, s, A, K, mol, cd;
};

constexpr Decoded decode(uint32_t u) noexcept
{
    const auto dec = [](uint32_t raw) -> int {
        return static_cast<int>(raw & 0x7u) - EXP_OFFSET;
    };
    return Decoded{
        static_cast<Type>    ((u >> BIT_T)   & 0x1u),
        static_cast<Numeric> ((u >> BIT_N)   & 0x3u),
        static_cast<Modifier>((u >> BIT_M)   & 0x3u),
        dec(u >> BIT_SR),
        dec(u >> BIT_RAD),
        dec(u >> BIT_M_),
        dec(u >> BIT_KG),
        dec(u >> BIT_S),
        dec(u >> BIT_A),
        dec(u >> BIT_K),
        dec(u >> BIT_MOL),
        dec(u >> BIT_CD)
    };
}

// readable dimension string, e.g. "kg·m²·s⁻³·A⁻¹" for volt.
inline std::string to_string(uint32_t u)
{
    const Decoded d = decode(u);
    if (d.type == Type::Digital) {
        return "digital(0x" + std::to_string(u) + ")";
    }
    struct Dim { const char* name; int exp; };
    const Dim dims[] = {
        {"sr",  d.sr}, {"rad", d.rad}, {"m",   d.m},
        {"kg",  d.kg}, {"s",   d.s},   {"A",   d.A},
        {"K",   d.K},  {"mol", d.mol}, {"cd",  d.cd}
    };
    const char* sup[] = {
        "⁻⁴","⁻³","⁻²","⁻¹","","","²","³"  // for exponents −4..+3
    };
    std::string out;
    for (const auto & x : dims) {
        if (x.exp == 0) continue;
        if (!out.empty()) out += "·";
        out += x.name;

        if (x.exp != 1) {
            const int idx = x.exp + EXP_OFFSET;  // 0..7
            out += sup[idx];
        }
    }
    return out.empty() ? "dimensionless" : out;
}

}  // namespace si_units
