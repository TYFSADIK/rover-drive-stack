// Host tests: c++ -std=c++17 -Ilib/convoy test/test_convoy.cpp -o /tmp/t && /tmp/t
#include <cassert>
#include <cstdio>
#include "follower.h"
#include "uart_frame.h"
using namespace convoy;

static StatePacket mk(uint16_t seq, float x) {
    StatePacket p; p.seq = seq; p.x_m = x; p.speed_mps = 1.0f; seal(p); return p;
}

int main() {
    // CRC rejects corruption; dedupe across links; seq wraparound
    StatePacket p = mk(1, 5); assert(valid(p)); p.x_m = 6; assert(!valid(p));
    assert(newer(1, 65535)); assert(!newer(65535, 1));
    Watchdog wd;
    assert(wd.on_packet(mk(10, 3), 0)); assert(!wd.on_packet(mk(10, 3), 5));   // same seq on 2nd link

    // State transitions by packet age
    assert(wd.step(500, 0, 0).mode == Mode::FOLLOW);
    Goal h = wd.step(2500, 0, 0); assert(h.mode == Mode::HOLD); assert(h.speed_cap < 1.0f && h.speed_cap > 0);
    assert(h.x > 3.0f);                                             // extrapolated forward
    assert(wd.step(4500, 0, 0).mode == Mode::SAFE_STOP);
    assert(wd.step(4500, 0, 0).speed_cap == 0.0f);
    assert(wd.on_packet(mk(11, 3.2f), 5000));
    assert(wd.step(5000, 0, 0).mode == Mode::FOLLOW);               // gap 3.2 within tolerance -> FOLLOW
    Watchdog wd2; wd2.on_packet(mk(1, 20), 0); wd2.step(5000, 0, 0); wd2.on_packet(mk(2, 20), 5100);
    assert(wd2.step(5100, 0, 0).mode == Mode::REJOIN);

    // Follower never closes inside the braking zone and ramps to stop
    Follower f; Pose self{0, 0, 0};
    for (int i = 0; i < 200; ++i) { Cmd c = f.step(self, {10, 0, 1.0f, Mode::FOLLOW}, 0.02f); self.x += c.v * 0.02f; }
    assert(self.x > 0.5f);
    float v = 1;
    for (int i = 0; i < 200; ++i) v = f.step(self, {self.x, 0, 0, Mode::SAFE_STOP}, 0.02f).v;
    assert(v == 0.0f);
    // UART framing: garbage before a frame, back-to-back frames, corrupted packet rejected
    {
        uint8_t buf[128]; size_t n = 0;
        buf[n++] = 0x00; buf[n++] = 0xAA; buf[n++] = 0x13;                 // noise
        n += encode_rx(mk(77, 4.0f), Link::WiFi, -71, buf + n);
        n += encode_pose({1, 2, 0.5f, 1.2f}, buf + n);
        StatePacket bad = mk(78, 1); bad.x_m = 9;                           // CRC now wrong
        n += encode_rx(bad, Link::LoRa, -90, buf + n);
        FrameParser fp; int rx = 0, pose = 0;
        for (size_t i = 0; i < n; ++i) {
            auto k = fp.push(buf[i]);
            if (k == FrameParser::Kind::Rx) { ++rx; assert(fp.pkt.seq == 77 && fp.link == Link::WiFi && fp.rssi == -71); }
            if (k == FrameParser::Kind::Pose) { ++pose; assert(fp.pose.speed == 1.2f); }
        }
        assert(rx == 1 && pose == 1);
    }
    std::puts("convoy host tests passed");
}
