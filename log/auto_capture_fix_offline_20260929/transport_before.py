"""RTDE receive-only connection is separate from explicit control acquisition."""
from collections import deque
import fcntl
from pathlib import Path
import threading
import time
import numpy as np
from .trajectory import vector


class Receiver:
    def __init__(self, ip, factory=None):
        if factory is None:
            from rtde_receive import RTDEReceiveInterface
            factory = RTDEReceiveInterface
        self.r = factory(ip, 500.)
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.latest = None
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True, name='rtde-feedback')
        self.thread.start()

    def _run(self):
        previous = None
        while not self.stop.is_set():
            try:
                ts = float(self.r.getTimestamp())
                if not np.isfinite(ts):
                    raise RuntimeError('Invalid robot timestamp')
                if previous is not None and ts < previous:
                    raise RuntimeError('Robot clock reset; restart and prepare a new preview')
                if ts != previous:
                    q, dq = vector(self.r.getActualQ()), vector(self.r.getActualQd())
                    mode, safety = int(self.r.getRobotMode()), int(self.r.getSafetyMode())
                    scale = float(self.r.getSpeedScaling())*float(self.r.getTargetSpeedFraction())
                    # Discard a torn read across RTDE packets; never re-stamp unchanged data.
                    if float(self.r.getTimestamp()) != ts:
                        continue
                    if not np.isfinite(scale) or not 0 <= scale <= 1.01:
                        raise RuntimeError('Invalid speed scaling')
                    with self.lock:
                        self.latest = dict(q=q, dq=dq, timestamp=ts, received=time.monotonic(),
                                           robot_mode=mode, safety_mode=safety, scaling=min(scale,1.))
                    previous = ts
            except Exception as exc:
                self.error = str(exc)
                break
            self.stop.wait(.001)

    def snapshot(self, timeout=.1):
        with self.lock:
            if self.error:
                raise RuntimeError(self.error)
            if self.latest is None or time.monotonic()-self.latest['received'] > timeout:
                raise RuntimeError('RTDE feedback is missing or stale')
            return {k:v.copy() if isinstance(v,np.ndarray) else v for k,v in self.latest.items()}

    def close(self):
        self.stop.set()
        self.thread.join(2)
        self.r.disconnect()


def local_control_conflicts():
    found = []
    for p in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            args = p.read_bytes().split(b'\0')
            # Executable/argv only: don't match shell scripts or this module's text.
            if any(Path(a.decode(errors='replace')).name in ('ur_ros2_control_node','ros2_control_node')
                   for a in args[:2]):
                found.append(p.parent.name)
        except (OSError, PermissionError):
            continue
    return found


class RTDEControl:
    def __init__(self, c):
        conflicts = local_control_conflicts()
        if conflicts:
            raise RuntimeError(f'UR/ros2_control still running (PIDs {conflicts}); perform control handoff first')
        self.file = open('/tmp/ur7e_auto_rtde_'+c['robot_ip'].replace('.','_')+'.lock','a')
        self.control = None
        try:
            fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            from rtde_control import RTDEControlInterface
            self.control = RTDEControlInterface(c['robot_ip'], c['frequency_hz'])
            if not self.control.setWatchdog(10.):
                raise RuntimeError('Could not enable controller-side watchdog')
        except Exception:
            self.close(c['stop_deceleration'])
            raise
        self.c = c

    def keepalive(self):
        if not self.control.kickWatchdog():
            raise RuntimeError('Control startup watchdog failed')

    def period(self):
        return self.control.initPeriod()

    def send(self, q, dt):
        if not self.control.servoJ(q.tolist(), 0., 0., dt, self.c['lookahead_time'], self.c['gain']):
            raise RuntimeError('servoJ returned false')
        # ur_rtde 1.6.3 watches input_int_register_0, which servoJ updates.
        # kickWatchdog writes NO_CMD to that same register. Sending it immediately
        # after servoJ can overwrite command 11 before the robot script reads it,
        # starving target updates. Keep watchdog kicks only in non-servo startup.

    def wait(self, started):
        self.control.waitPeriod(started)

    def close(self, deceleration):
        errors = []
        if self.control is not None:
            for name, args in [('servoStop',(deceleration,)),('stopScript',()),('disconnect',())]:
                try:
                    getattr(self.control,name)(*args)
                except Exception as exc:
                    errors.append(f'{name}: {exc}')
            self.control = None
        if getattr(self,'file',None):
            self.file.close()
            self.file = None
        return errors


class Servo:
    """Single writer at 500 Hz; ROS calls and CUDA run outside this thread/process."""
    def __init__(self, receiver, control, config, q0):
        self.receiver, self.control, self.c = receiver, control, config
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.finished = threading.Event()
        self.qcmd = vector(q0).copy()
        self.segment = None
        self.segment_time = 0.
        self.segment_done = True
        self.error = None
        self.heartbeat = time.monotonic()
        self.trace = deque(maxlen=300000)
        self.cleanup_errors = []
        self.thread = threading.Thread(target=self._run, daemon=True, name='rtde-servo')
        self.thread.start()

    def ping(self):
        self.heartbeat = time.monotonic()
        if self.error:
            raise RuntimeError(self.error)
        if self.finished.is_set():
            raise RuntimeError('Servo thread stopped')

    def move(self, segment):
        with self.lock:
            if not self.segment_done or np.max(abs(self.qcmd-segment.a)) > 1e-8:
                raise ValueError('Discontinuous segment or previous segment unfinished')
            self.segment, self.segment_time, self.segment_done = segment, 0., False

    def _run(self):
        dt, previous, scale = 1/self.c['frequency_hz'], None, 0.
        try:
            while not self.stop.is_set():
                now = time.monotonic()
                if previous is not None and now-previous > self.c['cycle_timeout_sec']:
                    raise RuntimeError(f"Servo scheduling deadline missed: interval={now-previous:.6f}s "
                                       f"limit={self.c['cycle_timeout_sec']:.6f}s; no catch-up jump")
                if now-self.heartbeat > .5:
                    raise RuntimeError('Coordinator heartbeat lost')
                previous = now
                token = self.control.period()
                state = self.receiver.snapshot(self.c['feedback_timeout_sec'])
                if state['robot_mode'] != 7 or state['safety_mode'] not in (1,2):
                    raise RuntimeError('Robot not running / safety stop; no automatic recovery')
                if np.max(abs(state['dq'])) > 1.5*self.c['max_joint_speed']+.02:
                    raise RuntimeError('Measured joint speed exceeds low-speed guard')
                if state['scaling'] < .01:
                    raise RuntimeError('Speed scaling is zero; restart with a new preview')
                scale_step = self.c['max_joint_acceleration']/(2*self.c['max_joint_speed'])*dt
                scale += float(np.clip(state['scaling']-scale,-scale_step,scale_step))
                with self.lock:
                    if np.max(abs(state['q']-self.qcmd)) > self.c['tracking_error_rad']:
                        raise RuntimeError('Joint tracking error exceeds limit')
                    if self.segment is not None and not self.segment_done:
                        # Fixed bounded increments; a delayed cycle cannot skip trajectory samples.
                        self.segment_time = min(self.segment.duration, self.segment_time+dt*scale)
                        self.qcmd = self.segment.sample(self.segment_time)
                        self.segment_done = self.segment_time >= self.segment.duration
                    q = self.qcmd.copy()
                if self.stop.is_set():
                    break
                self.control.send(q,dt)
                self.trace.append([now, state['timestamp'], scale, *q.tolist(), *state['q'].tolist(), *state['dq'].tolist()])
                self.control.wait(token)
        except Exception as exc:
            self.error = str(exc)
        finally:
            self.cleanup_errors = self.control.close(self.c['stop_deceleration'])
            self.finished.set()

    def close(self):
        self.stop.set()
        self.thread.join(5)
        if self.thread.is_alive():
            raise RuntimeError('Control thread did not stop; controller watchdog remains the fallback')
        if self.cleanup_errors:
            raise RuntimeError('Stop acknowledgement failed: '+str(self.cleanup_errors))
