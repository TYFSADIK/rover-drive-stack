// T4 blackout run on the host: lead drives at 1 m/s, both links drop from t=10 s to t=16 s.
// c++ -std=c++17 -Ifirmware/lib/convoy sim/blackout_sim.cpp -o /tmp/sim && /tmp/sim > data/blackout.csv
#include <cmath>
#include <cstdio>
#include "follower.h"
using namespace convoy;

int main() {
    Watchdog wd; Follower fol; Pose me{-3, 0, 0};
    float lx = 0; uint16_t seq = 0;
    const char* names[] = {"FOLLOW", "HOLD", "SAFE_STOP", "REJOIN"};
    std::printf("t_s,lead_x,follower_x,gap_m,mode,v\n");
    for (uint32_t t = 0; t <= 30000; t += 20) {
        bool lead_moving = t < 12000 || t > 18000;
        if (lead_moving) lx += 1.0f * 0.02f;
        bool blackout = t >= 10000 && t < 16000;
        if (t % 100 == 0 && !blackout) {
            StatePacket p; p.seq = ++seq; p.t_ms = t; p.x_m = lx; p.speed_mps = lead_moving ? 1.0f : 0.0f; seal(p);
            wd.on_packet(p, t);
        }
        Goal g = wd.step(t, me.x, me.y);
        Cmd c = fol.step(me, g, 0.02f);
        me.x += c.v * std::cos(me.heading) * 0.02f; me.heading += c.w * 0.02f;
        if (t % 500 == 0) std::printf("%.1f,%.2f,%.2f,%.2f,%s,%.2f\n", t / 1000.0, lx, me.x, lx - me.x, names[int(g.mode)], c.v);
    }
}
