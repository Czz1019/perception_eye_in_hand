"""Small deterministic state rules, also exercised without ROS/hardware."""
import numpy as np


def execution_gate(c, frozen, trajectory, state, now_ros, now_wall, confirmed, tf_ok, calibration_ok):
    if not c['execution_enabled']:raise ValueError('execution_enabled=false (preview only)')
    if not c['commissioned'] or not confirmed:raise ValueError('Commissioning and current static-scene confirmation are required')
    if frozen is None or trajectory is None:raise ValueError('No frozen map or validated plan')
    if trajectory.map_uid!=frozen.uid:raise ValueError('Plan map version mismatch')
    if state is None or now_wall-state[3]>c['joint_timeout'] or abs(now_ros-state[2])>c['joint_timeout']:
        raise ValueError('Stale joint state')
    if np.max(abs(state[1]))>c['settle_speed']:raise ValueError('Robot must be stationary before execution')
    if np.max(abs(state[0]-trajectory.positions[0]))>c['start_tolerance']:raise ValueError('Robot moved since planning; replan')
    if not tf_ok:raise ValueError('Live TF does not match commissioned model')
    if not calibration_ok:raise ValueError('Camera calibration differs from frozen map')
    # Deliberately no age test on static ESDF. Its validity is scene/model/session based.
    return True
