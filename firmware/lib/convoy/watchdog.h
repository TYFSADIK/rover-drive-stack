// The only component allowed to decide what the motors do.
//   FOLLOW    packet age < 1.0 s   track the newest packet
//   HOLD      1.0 s .. 4.0 s       track an extrapolated lead pose, ramping speed down
//   SAFE_STOP > 4.0 s              controlled stop, wait
//   REJOIN    first fresh packet after SAFE_STOP/HOLD; close the gap slowly, then FOLLOW
#pragma once
#include <cmath>
#include <cstdint>
#include "packet.h"

namespace convoy {

enum class Mode : uint8_t { FOLLOW, HOLD, SAFE_STOP, REJOIN };

struct WatchdogConfig {
    uint32_t follow_max_age_ms = 1000;
    uint32_t grace_ms = 4000;           // safety parameter: tune like it drives a vehicle
    float    rejoin_speed_cap = 0.3f;   // m/s while rejoining
    float    gap_tolerance_m = 0.5f;
    float    desired_gap_m = 3.0f;
};

struct Goal { float x, y, speed_cap; Mode mode; };

class Watchdog {
public:
    explicit Watchdog(WatchdogConfig c = {}) : cfg_(c) {}

    // Feed every valid packet from any link. Duplicates / old packets are dropped.
    bool on_packet(const StatePacket& p, uint32_t now_ms) {
        if (!valid(p)) return false;
        if (have_ && !newer(p.seq, last_.seq)) return false;
        bool was_degraded = !have_ || mode_ == Mode::HOLD || mode_ == Mode::SAFE_STOP;
        last_ = p; rx_ms_ = now_ms; have_ = true;
        if (was_degraded) mode_ = Mode::REJOIN;
        return true;
    }

    Goal step(uint32_t now_ms, float self_x, float self_y) {
        if (!have_) return {self_x, self_y, 0.0f, mode_ = Mode::SAFE_STOP};
        uint32_t age = now_ms - rx_ms_;
        if (age > cfg_.grace_ms) mode_ = Mode::SAFE_STOP;
        else if (age > cfg_.follow_max_age_ms && mode_ != Mode::SAFE_STOP) mode_ = Mode::HOLD;
        else if (mode_ == Mode::REJOIN) {
            float gap = std::hypot(last_.x_m - self_x, last_.y_m - self_y);
            if (std::fabs(gap - cfg_.desired_gap_m) < cfg_.gap_tolerance_m) mode_ = Mode::FOLLOW;
        }
        switch (mode_) {
            case Mode::FOLLOW:
                return {last_.x_m, last_.y_m, last_.speed_mps * 1.2f, mode_};
            case Mode::REJOIN:
                return {last_.x_m, last_.y_m, cfg_.rejoin_speed_cap, mode_};
            case Mode::HOLD: {
                // Extrapolate, and ramp the speed cap linearly to 0 across the grace window.
                float dt = age / 1000.0f;
                float ramp = 1.0f - float(age - cfg_.follow_max_age_ms) /
                                    float(cfg_.grace_ms - cfg_.follow_max_age_ms);
                return {last_.x_m + std::cos(last_.heading_rad) * last_.speed_mps * dt,
                        last_.y_m + std::sin(last_.heading_rad) * last_.speed_mps * dt,
                        last_.speed_mps * std::fmax(ramp, 0.0f), mode_};
            }
            case Mode::SAFE_STOP:
            default:
                return {self_x, self_y, 0.0f, mode_};
        }
    }

    Mode mode() const { return mode_; }
    const WatchdogConfig& config() const { return cfg_; }

private:
    WatchdogConfig cfg_;
    StatePacket last_{};
    uint32_t rx_ms_ = 0;
    bool have_ = false;
    Mode mode_ = Mode::SAFE_STOP;
};

}  // namespace convoy
