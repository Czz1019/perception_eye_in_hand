"""Final-goal SAGE, persistent shifted horizon, asynchronous bounded execution.

Search is an optional process-local helper, never a gate for local optimization.
"""
import time,signal
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
import numpy as np
from ur7e_real_static.trajectory import edge_clear
from .online import DynamicBlocked


def background_route(config,snapshot,start,goal,anchors):
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    from ur7e_real_static.model import Geometry
    from .planning import route,shortcut
    g=Geometry(config['robot_urdf'],config['collision_spheres'])
    c=dict(config,max_plan_seconds=10.)
    return shortcut(g,snapshot,route(g,snapshot,start,goal,anchors,c),c)


def checked_prefix(g,snapshot,start,velocities,dt,margin,self_margin):
    """A rejected later edge does not invalidate the earlier checked prefix."""
    points=[np.array(start,copy=True)]
    for velocity in velocities:
        nxt=points[-1]+np.asarray(velocity)*dt
        if not edge_clear(g,snapshot,points[-1],nxt,margin,self_margin):break
        points.append(nxt)
    return points


def shift_count(now,origin,dt,horizon):
    steps=max(0,int((now-origin+1e-9)//dt))
    return min(steps,horizon),origin+steps*dt


def reach(n,task,goal,token):
    if n.frozen.snapshot.query(np.asarray(goal)[None])[0]<=0:
        raise ValueError('Goal must lie in observed or tabletop-supported space')
    dt=.05;speed=n.scan['rolling_speed'];period=.04
    begun=time.monotonic();last_progress=begun;last_search=begun
    last_blocked=None
    best_error=np.inf;guide=None;future=None;pool=None;guide_index=1
    n.continuous_active=True;n.phase='REACHING';n.error=None
    n.event('local_goal_started',revision=token,goal=goal.tolist())
    try:
        while not n.exit.is_set() and not n.stopped and token==n.revision:
            if time.monotonic()-begun>240:raise TimeoutError('Continuous reaching budget exceeded')
            if task.future is not None and not task.future.done():
                # Servo independently checks age and active trajectories. Do not
                # repeatedly copy maps/check guides while the sole job is in flight.
                n.exit.wait(.003);continue
            wall=time.monotonic()
            if wall-begun>240:raise TimeoutError('Continuous reaching budget exceeded')
            state=n.state();q,dq=state[:2]
            try:snapshot,plan_version=n.read_online() if n.online else (n.collision_snapshot,0)
            except DynamicBlocked:
                with n.lock:n.segment=None;n.pending_segments=[]
                n.phase='WAIT_DYNAMIC';n.error='ONLINE_MAP_STALE';n.exit.wait(.02);continue
            error=float(np.linalg.norm(n.g.fk(q)['tool0'][:3,3]-goal))
            if error<.008:
                with n.lock:n.segment=None;n.pending_segments=[]
                n.phase='GOAL_REACHED';n.error=None
                n.event('goal_reached',goal=goal.tolist(),error_m=error,iterations=n.iterations,revision=token)
                import json
                (n.output/'sage_feedback.json').write_text(json.dumps(n.feedback_used))
                return
            if error<best_error-.003:best_error=error;last_progress=wall
            if future is not None and future.done():
                try:
                    guide=future.result();guide_index=1
                    n.event('background_guide',revision=token,joints=[v.tolist() for v in guide])
                except Exception as exc:n.event('background_failed',revision=token,error=str(exc))
                future=None
            # No helper for a physically occupied goal. Do not burn 30 s on its IK.
            blocked=snapshot.query(np.asarray(goal)[None])[0]<=0
            if blocked!=last_blocked:
                n.event('goal_occupancy',revision=token,blocked=bool(blocked),map_version=plan_version);last_blocked=blocked
            if not blocked and wall-last_progress>2. and wall-last_search>5. and future is None:
                if pool is None:pool=ProcessPoolExecutor(max_workers=1,mp_context=get_context('spawn'))
                future=pool.submit(background_route,n.c,snapshot,q.copy(),goal.copy(),n.frozen.metadata['views'])
                last_search=wall;n.event('background_requested',revision=token,map_version=plan_version)
            guide_q=None
            if guide is not None:
                while guide_index<len(guide)-1 and np.max(abs(q-guide[guide_index]))<.06:guide_index+=1
                if guide_index<len(guide) and edge_clear(n.g,snapshot,q,guide[guide_index],n.c['collision_margin'],n.c['self_margin']):
                    guide_q=guide[guide_index].copy()
            task.submit(q.copy(),dq.copy(),n.now(),snapshot,plan_version,goal.copy(),token,guide_q)
            result=task.poll()
            if result is None:
                n.exit.wait(.005);continue
            n.iterations+=1
            if token!=n.revision or n.stopped:return
            if result['revision']!=token:continue
            latency=time.monotonic()-result['wall'];shifts=result['shift_steps'];actions=result['actions'];plan_version=result['map_version']
            if latency>.3:
                n.event('stale_plan',latency=latency,optimize=result['optimize'],revision=token);continue
            # Goal blockage permits perception and optimization, but prevents execution.
            if blocked:
                with n.lock:n.segment=None;n.pending_segments=[]
                n.phase='WAIT_DYNAMIC';n.error='ONLINE_GOAL_BLOCKED';n.exit.wait(max(0.,period-latency));continue
            try:latest,execute_version=n.read_online() if n.online else (snapshot,0)
            except DynamicBlocked:continue
            velocities=np.clip(actions[:4],-speed,speed)
            # Near the goal, reduce command speed without changing obstacle margins.
            velocities*=min(1.,max(.1,error/.05))
            guard_started=time.monotonic()
            with n.lock:
                # Anchor and install atomically: the servo cannot advance the command
                # between validation and replacement, avoiding backward command jumps.
                actual=n.state()[0].copy();command=n.qcmd.copy()
                points=checked_prefix(n.g,latest,command,velocities,dt,n.c['collision_margin'],n.c['self_margin'])
                accepted=len(points)>=2 and edge_clear(n.g,latest,actual,points[1],n.c['collision_margin'],n.c['self_margin'])
                if accepted:
                    n.set_segment(command,points[1],dt,False,token)
                    n.pending_segments=[(a.copy(),b.copy(),0.,dt,False) for a,b in zip(points[1:-1],points[2:])]
                    n.plan_valid_until=time.monotonic()+.3
            if not accepted:
                n.event('local_step_rejected',revision=token,map_version=execute_version)
                n.exit.wait(max(0.,period-(time.monotonic()-wall)));continue
            n.phase='REACHING';n.error=None
            record=dict(plan_map_version=plan_version,execute_map_version=execute_version,iteration=n.iterations,stamp=n.state()[2],q=actual.tolist(),dq=dq.tolist(),next=points[1].tolist(),revision=token,shift_steps=shifts,latency=latency,optimize_seconds=result['optimize'],prefix=[p.tolist() for p in points],background_pending=future is not None,goal_error=error,guard_seconds=time.monotonic()-guard_started)
            n.feedback_used.append(record)
            import json
            n.feedback_log.write(json.dumps(record)+'\n');n.feedback_log.flush()
            n.exit.wait(max(0.,period-(time.monotonic()-wall)))
    finally:
        with n.lock:n.continuous_active=False;n.segment=None;n.pending_segments=[]
        if pool is not None:pool.shutdown(wait=False,cancel_futures=True)
