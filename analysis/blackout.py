"""Score a blackout run CSV (from sim/blackout_sim.cpp or real follower telemetry).

    python -m analysis.blackout data/blackout.csv
"""
import csv
import sys


def score(path, min_safe_gap_m=1.8):
    rows = list(csv.DictReader(open(path)))
    gaps = [float(r["gap_m"]) for r in rows]
    modes = [r["mode"] for r in rows]
    t = [float(r["t_s"]) for r in rows]
    transitions = [(t[i], modes[i - 1], modes[i]) for i in range(1, len(modes)) if modes[i] != modes[i - 1]]
    rejoin_t = next((tt for tt, a, b in transitions if b == "FOLLOW" and a == "REJOIN"), None)
    stop_t = next((tt for tt, a, b in transitions if b == "SAFE_STOP"), None)
    return {"min_gap_m": round(min(gaps[5:]), 2), "safe": min(gaps[5:]) >= min_safe_gap_m,
            "transitions": transitions, "safe_stop_at_s": stop_t, "back_to_follow_s": rejoin_t}


if __name__ == "__main__":
    for k, v in score(sys.argv[1]).items():
        print(f"{k}: {v}")
