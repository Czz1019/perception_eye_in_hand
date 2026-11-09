import numpy as np
import pytest

from ur7e_nvblox.sampling import UpdateBudget, advance_deadline, native_grid_points


def test_native_centres_follow_map_origin_and_resolution_not_roi_origin():
    center, size = np.array([0.13, -0.04, 0.51]), np.array([2., 2., 1.5])
    for voxel, expected in ((0.05, 48000), (0.1, 6000)):
        points = native_grid_points(center, size, voxel)
        assert len(points) == expected
        np.testing.assert_allclose(points / voxel - .5, np.round(points / voxel - .5), atol=3e-6)
        assert (points >= center - size / 2 - 1e-6).all()
        assert (points < center + size / 2 + 1e-6).all()


def test_roi_limit_checked_before_allocating_meshgrid():
    with pytest.raises(ValueError, match='2 million'):
        native_grid_points([0, 0, 0], [100, 100, 100], .01)
    with pytest.raises(ValueError, match='no voxel centres'):
        native_grid_points([0, 0, 0], [.001, .001, .001], .1)


def test_rate_deadline_preserves_phase_without_catchup_bursts():
    # 8 ms processing must not turn a 30 Hz schedule into 24 Hz.
    deadline = 10 + 1 / 30
    for index in range(1, 100):
        deadline = advance_deadline(deadline, 10 + index / 30 + .008, 30)
        assert deadline == pytest.approx(10 + (index + 1) / 30)
    assert advance_deadline(1., 5.01, 30) > 5.01


def test_update_budget_accepts_jittered_30hz_images_without_unbounded_burst():
    budget = UpdateBudget(30)
    times = np.arange(120)/30 + .006*np.sin(np.arange(120))
    assert sum(budget.ready(float(t)) for t in times) >= 119
    # Long sensor silence may allow two distinct new images, never a backlog.
    assert sum(budget.ready(100.) for _ in range(100)) == 2
