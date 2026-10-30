// Framing on the Pi <-> ESP32 UART link (both directions), portable and host-tested.
//   ESP32 -> Pi : AA 55 'R' link rssi(int8) StatePacket(26)          received packet, already CRC-checked + deduped
//   Pi -> ESP32 : AA 55 'P' x(f32) y(f32) heading(f32) speed(f32)     lead pose to broadcast
#pragma once
#include <cstdint>
#include <cstring>
#include "packet.h"

namespace convoy {

enum class Link : uint8_t { LoRa = 0, WiFi = 1 };
constexpr uint8_t kSync0 = 0xAA, kSync1 = 0x55;
constexpr size_t kRxFrameLen = 5 + sizeof(StatePacket);
constexpr size_t kPoseFrameLen = 3 + 16;

struct Pose4 { float x, y, heading, speed; };

inline size_t encode_rx(const StatePacket& p, Link link, int8_t rssi, uint8_t* out) {
  out[0] = kSync0; out[1] = kSync1; out[2] = 'R'; out[3] = uint8_t(link); out[4] = uint8_t(rssi);
  std::memcpy(out + 5, &p, sizeof(p));
  return kRxFrameLen;
}

inline size_t encode_pose(const Pose4& q, uint8_t* out) {
  out[0] = kSync0; out[1] = kSync1; out[2] = 'P';
  std::memcpy(out + 3, &q, sizeof(q));
  return kPoseFrameLen;
}

// Byte-at-a-time parser that resynchronises on AA 55 after any garbage.
class FrameParser {
 public:
  enum class Kind { None, Rx, Pose };
  Kind push(uint8_t b) {
    if (n_ == 0 && b != kSync0) return Kind::None;
    if (n_ == 1 && b != kSync1) { n_ = (b == kSync0); return Kind::None; }
    buf_[n_++] = b;
    if (n_ == 3 && b != 'R' && b != 'P') { n_ = 0; return Kind::None; }
    const size_t need = n_ >= 3 ? (buf_[2] == 'R' ? kRxFrameLen : kPoseFrameLen) : 0;
    if (need && n_ == need) {
      n_ = 0;
      if (buf_[2] == 'R') {
        std::memcpy(&pkt, buf_ + 5, sizeof(pkt));
        link = Link(buf_[3]); rssi = int8_t(buf_[4]);
        return valid(pkt) ? Kind::Rx : Kind::None;
      }
      std::memcpy(&pose, buf_ + 3, sizeof(pose));
      return Kind::Pose;
    }
    return Kind::None;
  }
  StatePacket pkt{};
  Pose4 pose{};
  Link link = Link::LoRa;
  int8_t rssi = 0;

 private:
  uint8_t buf_[kRxFrameLen]{};
  size_t n_ = 0;
};

}  // namespace convoy
