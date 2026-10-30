// Lead -> follower state packet. 26 bytes, little-endian, CRC-16/CCITT protected.
// Same bytes go out on LoRa (915 MHz) and Wi-Fi (ESP-NOW/UDP); the follower de-duplicates by seq.
#pragma once
#include <cstdint>
#include <cstring>

namespace convoy {

#pragma pack(push, 1)
struct StatePacket {
    uint8_t  magic = 0xC7;
    uint8_t  vehicle_id = 1;
    uint16_t seq = 0;
    uint32_t t_ms = 0;        // lead's clock
    float    x_m = 0, y_m = 0; // lead pose in shared odom frame
    float    heading_rad = 0;
    float    speed_mps = 0;
    uint16_t crc = 0;
};
#pragma pack(pop)
static_assert(sizeof(StatePacket) == 26, "packet layout changed");

inline uint16_t crc16_ccitt(const uint8_t* d, size_t n, uint16_t crc = 0xFFFF) {
    for (size_t i = 0; i < n; ++i) {
        crc ^= uint16_t(d[i]) << 8;
        for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

inline void seal(StatePacket& p) {
    p.crc = crc16_ccitt(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - 2);
}

inline bool valid(const StatePacket& p) {
    return p.magic == 0xC7 &&
           p.crc == crc16_ccitt(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - 2);
}

// Sequence comparison that survives uint16 wraparound.
inline bool newer(uint16_t a, uint16_t b) { return int16_t(a - b) > 0; }

}  // namespace convoy
