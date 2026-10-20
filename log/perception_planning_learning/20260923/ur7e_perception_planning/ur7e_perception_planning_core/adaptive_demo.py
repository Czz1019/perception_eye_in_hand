"""Observe only until a checked route exists, then track it with online SAGE."""
import json
import time
import numpy as np
import torch
from .demo import scan_configuration, guide_is_clear, velocity_proposal
from .routing import goal_route, joint_route, recovery_step
from .runtime import GPU_LOCK


def run_demo(node, task):
    from .planner import check_segment
    views, scan_cfg = scan_configuration()
    rollout = task.controller.rollout_fn
    durations = rollout.dynamics_model.traj_dt.detach().cpu().numpy()
    scan_index, epoch, processed = 0, None, -1
    route, segment_start, mode = None, None, 'goal'
    dwell_started = dwell_sequence = hold = None
    retry_after, check_after, checked_sequence = 0., 0., -1
    node.completed_views = 0
    has_reached_goal = False
    while not node.stop.is_set():
        started = time.monotonic()
        state, snapshot, goal, generation = node.inputs()
        if node.freshness(state, snapshot):
            node.stop.wait(.05); continue
        q, dq = state[:2]
        if epoch is None:
            if np.max(abs(q-views[0])) > .08:
                node.demo_phase = node.status = 'DEMO_REQUIRES_INITIAL_POSE'
                node.stop.wait(.2); continue
            epoch = snapshot.epoch
        if epoch != snapshot.epoch:
            node.demo_phase = node.status = 'DEMO_RESTART_AFTER_MAP_RESET'
            node.stop.wait(.2); continue
        node.ee = node.geometry.fk(q)['tool0'][:3, 3]
        if goal is None:
            with node.lock:
                if node.goal is None:
                    node.goal = np.asarray(node.cfg['demo_goal'])
                goal = node.goal.copy()
        if generation != processed:
            processed = generation
            route = hold = dwell_started = None
            mode, retry_after = 'goal', 0.
        node.scan_index = scan_index
        node.demo_phase = (('REACHING' if route is not None else 'CHECKING_COVERAGE')
                           if mode == 'goal' else f'SCANNING_{scan_index+1}_OF_{len(views)}')
        if mode == 'goal' and np.linalg.norm(goal-node.ee) < .012:
            has_reached_goal = True
            if hold is None:
                hold = q.copy()
            with node.lock:
                if generation == node.generation:
                    node.target = (hold, time.monotonic(), snapshot.epoch, generation)
            node.status = node.demo_phase = 'GOAL_REACHED'
            node.stop.wait(.1); continue

        if route is not None and snapshot.sequence != checked_sequence and time.monotonic() >= check_after:
            checked_sequence, check_after = snapshot.sequence, time.monotonic()+1.
            remaining = [q, *route]
            clear = (check_segment(node.geometry, snapshot, q, route[0], node.cfg['collision_margin'])[0]
                     if mode == 'recover' else
                     all(guide_is_clear(node.geometry, snapshot, a, b) for a, b in zip(remaining, remaining[1:])))
            if not clear:
                route = None; retry_after = 0.
                with node.lock:
                    node.target = None
                node.status = 'REPLANNING'
                node.get_logger().info('Updated perception invalidated the remaining route; replanning')

        if route is None:
            if mode == 'recover':
                mode = 'goal'
            if time.monotonic() < retry_after:
                node.stop.wait(.05); continue
            with node.lock:
                node.target = None
            node.status = 'SOLVING_GUIDE'
            if mode == 'goal':
                candidate = goal_route(node.geometry, snapshot, q, goal, views, budget=1.,
                                       allow_rrt=has_reached_goal or scan_index >= len(views))
                recovery = recovery_step(node.geometry, snapshot, q, node.cfg['collision_margin']) if candidate is None else None
                if recovery is not None:
                    mode = 'recover'; candidate = [q, recovery]
                    node.get_logger().info('Restoring planning clearance with a checked short retreat')
                elif candidate is None and scan_index < len(views):
                    mode = 'observe'
                    dwell_started = None
                elif candidate is None:
                    node.status = 'GOAL_NOT_OBSERVED_OR_BLOCKED'
                    retry_after = time.monotonic()+3.
                    continue
            if mode == 'observe':
                candidate = joint_route(node.geometry, snapshot, q, [views[scan_index]], views, budget=1.)
                if candidate is None:
                    node.status = 'OBSERVATION_ROUTE_BLOCKED'
                    # A later map may admit the goal directly, even if a
                    # particular observation pose remains inaccessible.
                    mode = 'goal'
                    retry_after = time.monotonic()+3.
                    continue
            if generation != node.generation:
                continue
            segment_start, route = q.copy(), [np.asarray(p) for p in candidate[1:]]
            checked_sequence, check_after = snapshot.sequence, time.monotonic()+1.
            node.route_waypoints = len(route)
            record = dict(goal=goal.tolist(), start=q.tolist(), end=route[-1].tolist(),
                          waypoints=[p.tolist() for p in candidate], mode=mode,
                          completed_views=node.completed_views, epoch=snapshot.epoch, sequence=snapshot.sequence)
            (node.run_dir/'goal_guide.json').write_text(json.dumps(record, indent=2))
            with (node.run_dir/'routes.jsonl').open('a') as log:
                log.write(json.dumps(record)+'\n')
            if mode == 'goal':
                node.get_logger().info(f'Goal corridor ready after {node.completed_views}/{len(views)} views; '
                                       f'{len(route)} SAGE segments for {goal.tolist()}')
        desired_q = route[0]
        if mode == 'recover' and np.max(abs(q-desired_q)) < .006:
            mode, route, retry_after = 'goal', None, 0.
            continue
        if len(route) > 1 and np.max(abs(q-desired_q)) < .012:
            route.pop(0); desired_q = route[0]; segment_start = q.copy()
            # Check the actual corner cut, not just the ideal waypoint.
            if not guide_is_clear(node.geometry, snapshot, q, desired_q):
                route = None; retry_after = 0.; continue
        if mode == 'observe' and len(route) == 1 and np.max(abs(q-desired_q)) < .015:
            if dwell_started is None:
                dwell_started, dwell_sequence, hold = time.monotonic(), snapshot.sequence, q.copy()
            with node.lock:
                if generation == node.generation:
                    node.target = (hold, time.monotonic(), snapshot.epoch, generation)
            node.status = 'SCAN_DWELL'
            if time.monotonic()-dwell_started >= scan_cfg['dwell_sec'] and snapshot.sequence-dwell_sequence >= 3:
                scan_index += 1; node.completed_views += 1
                route = hold = dwell_started = None
                mode, retry_after = 'goal', 0.
            node.stop.wait(.05); continue
        node.demo_phase = 'REACHING' if mode == 'goal' else f'SCANNING_{scan_index+1}_OF_{len(views)}'
        if mode == 'recover':
            node.demo_phase = 'RECOVERING_MARGIN'
        xyz = goal if mode == 'goal' and len(route) == 1 else node.geometry.fk(desired_q)['tool0'][:3, 3]
        speed = node.speed_limit(snapshot)
        dt = node.cfg['planning_dt']
        proposal = velocity_proposal(q, desired_q, durations, speed)
        current = np.r_[q, np.clip(dq, -node.cfg['max_joint_speed'], node.cfg['max_joint_speed']), np.zeros(6), state[2]]
        optimization_started = time.monotonic()
        with GPU_LOCK, torch.no_grad():
            task.update_params(goal_ee_pos=xyz, goal_ee_rot=node.geometry.fk(q)['tool0'][:3, :3])
            rollout.set_map(snapshot)
            rollout.joint_guide = torch.as_tensor(desired_q, **task.tensor_args)
            rollout.joint_guide_weight = 150.
            rollout.joint_segment = torch.as_tensor(np.stack((segment_start, desired_q)), **task.tensor_args)
            task.controller.mean_action = torch.as_tensor(proposal, **task.tensor_args)
            result = task.controller.optimize(torch.as_tensor(current, **task.tensor_args)[None], shift_steps=0)
            velocity = result[0][0].detach().cpu().numpy()
            if task.controller.top_trajs is not None:
                node.preview = task.controller.top_trajs[:3].detach().cpu().numpy().copy()
        node.optimize_ms = (time.monotonic()-optimization_started)*1000
        node.sage_count += 1
        candidate = q+np.clip(velocity, -speed, speed)*dt
        ok, _, _ = check_segment(node.geometry, snapshot, q, candidate, node.cfg['collision_margin'])
        candidate_clearance = np.min(snapshot.query(node.geometry.spheres(candidate))-node.geometry.radii)
        required = (np.min(snapshot.query(node.geometry.spheres(q))-node.geometry.radii)-1e-5
                    if mode == 'recover' else node.cfg['collision_margin']+.005)
        ok = (ok and candidate_clearance >= required
              and node.geometry.camera_axis_z(candidate)[0] <= -.23)
        with node.lock:
            if generation == node.generation and not node.freshness(*node.inputs()[:2]):
                safe_hold = q.copy() if node.q_command is None else node.q_command.copy()
                node.target = (candidate if ok else safe_hold, time.monotonic(), snapshot.epoch, generation)
                node.command_source = ('sage_scan' if mode == 'observe' else 'sage') if ok else 'blocked_hold'
        node.status = ('SCANNING' if mode == 'observe' else 'PLANNING') if ok else 'BLOCKED'
        node.plan_count += 1; node.last_plan_map = snapshot.sequence
        node.plan_ms = (time.monotonic()-started)*1000
        node.admission_ms = (time.monotonic()-optimization_started)*1000-node.optimize_ms
        node.stop.wait(max(.02, dt-(time.monotonic()-started)))
