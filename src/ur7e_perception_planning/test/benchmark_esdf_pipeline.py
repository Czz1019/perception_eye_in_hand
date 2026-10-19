"""Same-data CPU/CUDA microbenchmarks; no ROS devices or obstacle truth in production.

Compares query allocation/readback and conservative EDT scheduling independently
of Gazebo rendering/thermal load. It does not measure closed-loop arrival time.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch
import numpy as np
from realtime_nvblox.types import CameraIntrinsics
from ur7e_nvblox.depth import depth_calibration,known_distances
from ur7e_nvblox.esdf import EfficientBackend
from ur7e_perception_planning_core.grid import grid_points,observed_free,conservative_field


def distribution(values):
    return dict(mean_ms=float(np.mean(values)),p50_ms=float(np.percentile(values,50)),p95_ms=float(np.percentile(values,95)))


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--reference',required=True);args=p.parse_args()
    intr=CameraIntrinsics(160,90,120,120,80,45);depth=np.full((90,160),1.,np.float32)
    calibration=depth_calibration(intr);config={'voxel_size_m':.025,'max_integration_distance_m':3.}
    b=EfficientBackend(calibration,config);pose=np.eye(4);pose[2,3]=.2
    for _ in range(5):b.integrate_depth(depth,pose)
    b.update_esdf()
    _,dims,points=grid_points([.15,.05,.6],[1.5,1.4,1.3],.02)
    b.prepare_query(points);expected=b.query_esdf(points)
    np.testing.assert_allclose(b.query_prepared(),expected,atol=1e-6)
    times={'old_allocating_query':[],'prepared_query':[]}
    methods={'old_allocating_query':lambda:b.query_esdf(points),'prepared_query':b.query_prepared}
    for i in range(100):
        for name in (list(methods) if i%2 else list(methods)[::-1]):
            start=time.perf_counter();result=methods[name]();elapsed=(time.perf_counter()-start)*1000
            if i>=10:times[name].append(elapsed)
            np.testing.assert_allclose(result,expected,atol=1e-6)
    # Exercise the other author's unmodified backend directly, avoiding duplicate
    # ROS package names and the missing source_gazebo_env.sh in its launcher.
    spec=importlib.util.spec_from_file_location('reference_backend',Path(args.reference)/'realtime_nvblox/nvblox_backend.py')
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    reference=module.NvbloxBackend(calibration,config)
    for _ in range(5):reference.integrate_depth(depth,pose)
    reference.update_esdf()
    rp,rd=reference.observed_esdf_numpy();op,od=b.observed_esdf_numpy()
    np.testing.assert_allclose(op,rp,atol=1e-6);np.testing.assert_allclose(od,rd,atol=1e-6)
    sparse=[]
    for _ in range(30):
        start=time.perf_counter();b.observed_esdf_numpy();sparse.append((time.perf_counter()-start)*1000)
    # Deterministic acquisition at 15 Hz; newly observed free space grows. Exact
    # full-rate EDT is compared with a conservative 5 Hz expansion schedule.
    raw=np.full(len(points),100.,np.float32);known=np.zeros(len(points),bool)
    from ur7e_perception_planning_core.edt import CudaEDT
    sources=np.zeros(len(points),np.uint8);cache_old={};cache_new={'edt_function':CudaEDT()};old=[];new=[];deferred=0
    for i in range(90):
        moved=pose.copy();moved[0,3]=-.45+i*.01
        sources[observed_free(points,depth,moved,intr,.05+np.sqrt(3)*.02/2)]=1
        start=time.perf_counter();exact,_=conservative_field(raw,known,sources,points,dims,.02,cache_old);old.append((time.perf_counter()-start)*1000)
        with patch('ur7e_perception_planning_core.grid.time.monotonic',return_value=100.+i/15.):
            start=time.perf_counter();safe,_=conservative_field(raw,known,sources,points,dims,.02,cache_new,min_interval=.2);new.append((time.perf_counter()-start)*1000)
        assert np.all(safe<=exact+1e-6)
        deferred+=int(cache_new.get('deferred',False))
    report=dict(query_points=len(points),native_known_cells=int(known_distances(expected,b.esdf_unknown_distance).sum()),
        query={k:distribution(v) for k,v in times.items()},sparse_export=distribution(sparse),sparse_points=len(op),
        reference_sparse_matches=True,field_exact=distribution(old),field_conservative_scheduled=distribution(new),
        field_deferred_frames=deferred,all_scheduled_distances_no_greater_than_exact=True)
    Path(args.output).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':main()
