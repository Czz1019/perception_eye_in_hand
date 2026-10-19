"""Reproduce Python-thread contention; diagnostic, not a timing assertion."""
import argparse
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import numpy as np
import yaml
from ur7e_real_static.model import Geometry
from ur7e_gazebo_esdf_sage.self_filter import robot_depth_mask
from ur7e_perception_planning_core.self_filter import robot_depth_mask as reference


def main():
    parser=argparse.ArgumentParser();parser.add_argument('output');a=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    g=Geometry(root/'config/real_calibrated.urdf',root/'config/collision_spheres.yaml')
    q=yaml.safe_load((root/'config/scan.yaml').read_text())['views'][6]
    args=(np.ones((120,160)),g.fk(q)[g.camera],SimpleNamespace(fx=115.,fy=115.,cx=80.,cy=60.),g.spheres(q),g.radii,g.link_ids)
    stop=threading.Event();original_interval=sys.getswitchinterval()
    def busy():
        while not stop.is_set():
            for i in range(1000):x=i*i
    thread=threading.Thread(target=busy);thread.start();result=[]
    try:
        for name,fn,interval in [('reference',reference,.005),('batch',robot_depth_mask,.005),('batch_short_quantum',robot_depth_mask,.001)]:
            sys.setswitchinterval(interval);samples=[]
            for _ in range(3):
                t=time.perf_counter();cpu=time.thread_time();fn(*args)
                samples.append(dict(wall_ms=1000*(time.perf_counter()-t),cpu_ms=1000*(time.thread_time()-cpu)))
            result.append(dict(mode=name,samples=samples))
    finally:
        stop.set();thread.join();sys.setswitchinterval(original_interval)
    Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':main()
