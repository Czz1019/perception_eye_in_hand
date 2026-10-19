"""Conservative speed tiers for the static-scene demo, driven by measured delay."""
import numpy as np


def latency_speed(maximum, age, recent_delays, plan_ms):
    if not np.isfinite(age) or age > 1.5:
        return 0.
    if len(recent_delays) < 4:
        return min(maximum, .08)
    delay = max(age, max(recent_delays))
    cap = .08
    if delay <= .8 and plan_ms < 300:
        cap = .16
    if delay <= .45 and plan_ms < 150:
        cap = .24
    if len(recent_delays) >= 8 and delay <= .30 and plan_ms < 100:
        cap = .30
    return min(maximum, cap)
