# SmartData
#
# reference:
#   Fröhlich, A. A. (2018) "SmartData: an IoT-ready API for sensor networks",
#
# network packet layout 
#     Origin(x, y, z, d, t) | Unit | Value | Expiry | MAC
#
# 32-bit unit identifier
# dimensional SI encoding, see si_units.hpp
uint32 unit

# Value (scalar)
float64 value

# origin(x, y, z, d, t)
# spatial location where the data was produced
float64 origin_x
float64 origin_y
float64 origin_z

# device enumerator
uint32 origin_device

# production timestamp as microseconds since UNIX epoch
uint64 origin_time

# expiry 
# validity window in microseconds, relative to origin_time]
# after origin_time + expiry, the data must not be used.
uint64 expiry

# MAC 
# Poly1305-AES authentication code (out of scope)
uint8[16] mac
