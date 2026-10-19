"""One persistent SAGE process; bounded one-job mailbox, no servo/GPU GIL sharing."""
import time,signal
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
import numpy as np

_context=None

def initialize(config,directory,session):
    global _context
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    import torch
    from ur7e_real_static.model import Geometry
    from .task import build_task
    from .partial_map import attach_cost
    torch.set_flush_denormal(True)
    g=Geometry(config['robot_urdf'],config['collision_spheres'])
    task=build_task(g,config,directory)
    if config['planning_policy']=='bounded_unknown':attach_cost(task.controller.rollout_fn,session)
    _context=dict(task=task,g=g,revision=None,origin=0.,map_version=None)

def compute(job):
    import torch
    from .continuous import shift_count
    ctx=_context;task=ctx['task'];controller=task.controller;rollout=controller.rollout_fn
    if job['snapshot'] is not None:rollout.set_map(job['snapshot'])
    if job['revision']!=ctx['revision']:
        controller.reset();ctx['revision']=job['revision'];ctx['origin']=job['stamp']
        task.update_params(goal_ee_pos=job['goal'],goal_ee_rot=ctx['g'].fk(job['q'])['tool0'][:3,:3])
    shifts,ctx['origin']=shift_count(job['stamp'],ctx['origin'],.05,controller.horizon)
    rollout.joint_segment=None;rollout.joint_guide=None;rollout.joint_guide_weight=0.
    if job['guide'] is not None:
        rollout.joint_guide=torch.tensor(job['guide'],**task.tensor_args);rollout.joint_guide_weight=25.
    measured=torch.tensor(np.r_[job['q'],job['dq'],np.zeros(6),job['stamp']],**task.tensor_args)[None]
    started=time.monotonic()
    with torch.no_grad():actions=controller.optimize(measured,shift_steps=shifts)[0].detach().cpu().numpy()
    return dict(actions=actions,shift_steps=shifts,optimize=time.monotonic()-started,revision=job['revision'],stamp=job['stamp'],q=job['q'],dq=job['dq'],wall=job['wall'],map_version=job['version'])

class Planner:
    def __init__(self,config,directory,session):
        self.pool=ProcessPoolExecutor(max_workers=1,mp_context=get_context('spawn'),initializer=initialize,initargs=(config,str(directory),str(session)))
        self.future=None;self.map_version=None
    def submit(self,q,dq,stamp,snapshot,version,goal,revision,guide):
        if self.future is not None:return
        # Version -1 is the warm-up static map, 0 is the online layer's initial map.
        transfer=snapshot if self.map_version!=version else None
        self.future=self.pool.submit(compute,dict(q=q,dq=dq,stamp=stamp,snapshot=transfer,version=version,goal=goal,revision=revision,guide=guide,wall=time.monotonic()))
        self.map_version=version
    def poll(self):
        if self.future is None or not self.future.done():return None
        future=self.future;self.future=None;return future.result()
    def warm(self,q,dq,stamp,snapshot,goal):
        self.submit(q,dq,stamp,snapshot,-1,goal,-1,None)
        self.future.result(timeout=60);self.future=None
    def close(self):
        processes=list((self.pool._processes or {}).values())
        self.pool.shutdown(wait=False,cancel_futures=True)
        for process in processes:
            process.join(2)
            if process.is_alive():process.terminate();process.join(1)
            if process.is_alive():process.kill();process.join(1)
