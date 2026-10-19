"""Validate delayed computation only while a static-scene capture stays still."""
import numpy as np


def result_rejection(c,states,q,stamp,received,now,wall_now):
    budget=c['stationary_processing_timeout']
    if now-stamp>budget or wall_now-received>budget:return 'PROCESSING_TIMEOUT'
    if not states:return 'NO_FEEDBACK'
    latest=states[-1]
    if now-latest[2]>c['joint_timeout'] or wall_now-latest[3]>c['joint_timeout']:
        return 'STALE_FEEDBACK'
    history=[s for s in states if s[2]>=stamp-.1]
    times=np.array([s[2] for s in history])
    if len(times)<2 or times[0]>stamp or times[-1]<stamp or np.max(np.diff(times))>.1:
        return 'INCOMPLETE_STATIONARY_HISTORY'
    positions=np.array([s[0] for s in history]);speeds=np.array([s[1] for s in history])
    if np.max(abs(positions-q))>=.008 or np.max(abs(speeds))>c['settle_speed']:
        return 'MOVED_DURING_PROCESSING'
    return None
