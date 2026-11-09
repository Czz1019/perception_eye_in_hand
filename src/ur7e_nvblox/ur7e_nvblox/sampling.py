"""Bounded map-aligned sampling and wall-clock scheduling, independent of ROS."""
import math

import numpy as np


def native_grid_points(center, size, voxel_size):
    """Voxel centres inside a fixed world-frame ROI (half-open upper bound)."""
    center = np.asarray(center, dtype=np.float64)
    size = np.asarray(size, dtype=np.float64)
    if (center.shape != (3,) or size.shape != (3,) or
            not np.isfinite(center).all() or not np.isfinite(size).all() or
            (size <= 0).any() or not np.isfinite(voxel_size) or voxel_size <= 0):
        raise ValueError('Invalid native voxel grid geometry')
    lower, upper = center - size / 2, center + size / 2
    # Snap in the world frame, not relative to the ROI/camera. Small tolerance
    # only stabilizes exact integer boundaries against floating-point rounding.
    first = np.ceil(lower / voxel_size - 0.5 - 1e-9).astype(np.int64)
    stop = np.ceil(upper / voxel_size - 0.5 - 1e-9).astype(np.int64)
    counts = np.maximum(stop - first, 0)
    if (counts == 0).any():
        raise ValueError('Native query ROI contains no voxel centres along an axis')
    if np.prod(counts, dtype=np.int64) > 2_000_000:
        raise ValueError('Native query ROI exceeds 2 million samples; enlarge voxel_size_m or reduce grid_size_m')
    axes = [(np.arange(a, b, dtype=np.float64) + 0.5) * voxel_size
            for a, b in zip(first, stop)]
    return np.stack(np.meshgrid(*axes, indexing='ij'), axis=-1).reshape(-1, 3).astype(np.float32)


def advance_deadline(deadline, now, rate_hz):
    """Keep phase without adding processing time or replaying missed ticks."""
    period = 1.0 / rate_hz
    if deadline <= 0:
        return now + period
    return deadline + max(1, math.floor((now - deadline) / period) + 1) * period


class UpdateBudget:
    """A two-frame budget tolerates delivery jitter without an update backlog.

    A strict per-frame minimum interval drops alternate frames when a nominal
    30 Hz source delivers occasional pairs slightly less than 33 ms apart.
    Only distinct new images consume this bounded budget.
    """

    def __init__(self, rate_hz):
        self.rate_hz = rate_hz
        self.last_time = None
        self.tokens = 1.0

    def ready(self, now):
        if self.last_time is not None:
            self.tokens = min(2.0, self.tokens + max(0.0, now-self.last_time)*self.rate_hz)
        self.last_time = now
        if self.tokens < 1.0 - 1e-6:
            return False
        self.tokens -= 1.0
        return True
