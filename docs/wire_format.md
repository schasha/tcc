# SmartData wire format

The semantic gateway publishes each SmartData both on a ROS topic (for
debug) and on a UDP multicast bus that other SmartData systems can read.
This document fixes the byte layout used on the bus.

The format mirrors the envelope from Fröhlich (2018) §3.1, Fig. 1:

    Origin(x, y, z, d, t) | Unit | Value | Expiry | MAC

It is **not** the SDAV TSTP frame — the gateway does not implement the
TSTP control plane (interest, response, hop-by-hop trust). It only
emits the data envelope a transducer would carry in its payload. SDAV
adapters can subscribe to this stream directly because the envelope
fields are the same.

## Byte layout (72 bytes, little-endian, packed)

| Offset | Size | Field           | Type     | Notes                                         |
|--------|------|-----------------|----------|-----------------------------------------------|
| 0      | 4    | `unit`          | u32      | dimensional SI identifier (see `si_units.hpp`) |
| 4      | 8    | `value`         | f64      | IEEE-754 double                               |
| 12     | 8    | `origin_x`      | f64      | meters, frame = `target_frame`                |
| 20     | 8    | `origin_y`      | f64      |                                               |
| 28     | 8    | `origin_z`      | f64      |                                               |
| 36     | 4    | `origin_device` | u32      | element index (arrays) or 0                   |
| 40     | 8    | `origin_time`   | u64      | µs since UNIX epoch                           |
| 48     | 8    | `expiry`        | u64      | µs, relative to `origin_time`                 |
| 56     | 16   | `mac`           | u8[16]   | Poly1305-AES (out of scope — zeros for now)   |

Total: **72 bytes**.

## Endianness and packing

Wire bytes are little-endian. The C++ struct in `include/sd_wire.hpp`
uses `#pragma pack(push,1)` so `sizeof(SDWire) == 72` on any platform
the gateway targets (x86_64, arm64 — both little-endian). A static
assert in the header pins this.

There is no marshalling step: `to_wire(sd)` returns a `SDWire` with the
right fields, and `std::memcpy` it into the UDP payload. The receiver
does the inverse with `from_wire`.

## Transport

- Multicast group: **224.1.1.1**
- UDP port: **5000**
- TTL: 1 (single LAN segment by default)
- One SmartData per datagram (72-byte payload, no batching for now)

These match SDAV's `_MCAST_GRP` / `_MCAST_PORT` so SDAV transducers
listening on the same group can interpret the envelope without changes.

## What this format does *not* include

- TSTP control headers (interest, response codes, hop count)
- TSTP timestamp sync deltas
- batch framing — each datagram is one SmartData
- the `MAC` field is currently zero; integrity/auth is left for future
  work (the field is reserved in the layout so adding it later does not
  shift offsets)
