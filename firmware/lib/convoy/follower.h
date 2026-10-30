// Pure-pursuit steering + gap-keeping speed for a skid-steer follower.
#pragma once
#include <algorithm>
#include <cmath>
#include "watchdog.h"

namespace convoy {

struct Pose { float x, y, heading; };
struct Cmd { float v, w; };   // m/s, rad/s

struct FollowerConfig {
    float desired_gap_m = 3.0f, k_gap = 0.6f, max_v = 1.5f, max_w = 1.5f;
    float accel = 0.8f;        // m/s^2 ramp limit, also used for controlled stops
};

class Follower {
public:
    explicit Follower(FollowerConfig c = {}) : cfg_(c) {}

    Cmd step(const Pose& self, const Goal& g, float dt) {
        float dx = g.x - self.x, dy = g.y - self.y;
        float dist = std::hypot(dx, dy);
        float alpha = std::atan2(dy, dx) - self.heading;
        alpha = std::atan2(std::sin(alpha), std::cos(alpha));
        float v_target = 0.0f;
        if (g.mode != Mode::SAFE_STOP && dist > 0.05f) {
            v_target = std::clamp(cfg_.k_gap * (dist - cfg_.desired_gap_m) + g.speed_cap * 0.8f,
                                  0.0f, std::min(g.speed_cap, cfg_.max_v));
            if (dist < cfg_.desired_gap_m * 0.6f) v_target = 0.0f;   // never close inside the braking zone
        }
        float dv = std::clamp(v_target - v_, -cfg_.accel * dt, cfg_.accel * dt);
        v_ += dv;
        float curvature = dist > 0.05f ? 2.0f * std::sin(alpha) / dist : 0.0f;
        float w = std::clamp(v_ * curvature, -cfg_.max_w, cfg_.max_w);
        return {v_, w};
    }

    // Skid-steer mixing to left/right wheel speeds.
    static void mix(const Cmd& c, float track_m, float& left, float& right) {
        left = c.v - c.w * track_m / 2.0f;
        right = c.v + c.w * track_m / 2.0f;
    }

private:
    FollowerConfig cfg_;
    float v_ = 0.0f;
};

}  // namespace convoy
