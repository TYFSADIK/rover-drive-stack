# Rover Drive Stack: Convoy Robots on a Degraded Mesh

Private research repo. Two rovers follow a lead vehicle in convoy over 915 MHz LoRa with a Wi-Fi (ESP-NOW) fallback. They hold formation through link drops and recover when the link returns.

Full write-up: [`docs/project-page.md`](docs/project-page.md)

## What's here

| Path | Language | Purpose |
|---|---|---|
| `firmware/lib/convoy/packet.h` | C++17 | 26-byte state packet, CRC-16/CCITT, wrap-safe sequence numbers |
| `firmware/lib/convoy/watchdog.h` | C++17 | FOLLOW / HOLD / SAFE_STOP / REJOIN state machine (1.0 s and 4.0 s thresholds) |
| `firmware/lib/convoy/follower.h` | C++17 | Pure-pursuit steering, gap keeping, accel-limited stops, skid-steer mixing |
| `firmware/src/lead`, `firmware/src/follower` | C++ / Arduino | ESP32 + SX1276 (RadioLib) + ESP-NOW. Both links send the same bytes and the follower de-duplicates by sequence number |
| `sim/blackout_sim.cpp` | C++ | T4 blackout run on your laptop, with CSV output |
| `analysis/blackout.py` | Python | Scores a run: minimum gap, state transitions, time to SAFE_STOP and back to FOLLOW |

## Quickstart

```bash
make test        # host unit tests for the firmware logic
make sim         # simulate a 6 s total link blackout and score it
make flash-lead  # needs PlatformIO
```

## Next steps
- [ ] Wheel encoder + BNO085 odometry task on the follower (`self_pose`)
- [ ] Shared frame: initialise both rovers at surveyed start points, or use AprilTag alignment
- [ ] Log every packet RX (link, seq, RSSI, SNR) to SD and plot link-quality vs distance
- [ ] Add packet authentication (HMAC-SHA256 truncated tag, key on the device) so spoofed packets are rejected. See `secure-fleet-autonomy`
- [ ] Hardware e-stop independent of the ESP32

**Safety:** `grace_ms` is a safety parameter. Keep it short, and never test with people inside the braking zone.
