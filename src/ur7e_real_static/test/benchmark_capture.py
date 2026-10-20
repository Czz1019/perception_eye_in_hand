"""Same-map query and field benchmark, including the saved pre-upgrade FieldBuilder.

Uses native CUDA fusion and synthetic plane views; no hardware, ROS transport,
TF waiting, self filtering or controller execution is included in these times.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import time
import numpy as np
from realtime_nvblox.types import CameraIntrinsics
from ur7e_nvblox.depth import depth_calibration,known_distances
from ur7e_nvblox.esdf import EfficientBackend
from ur7e_real_static.field import FieldBuilder
from fixtures import config


def summarize(values):
    return dict(p50_ms=float(np.median(values)),p95_ms=float(np.percentile(values,95)))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--baseline-field',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    spec=importlib.util.spec_from_file_location('baseline_field',args.baseline_field)
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    c,_=config(out/'fixture');c['grid_size']=[2.,2.,1.4]
    before=module.FieldBuilder(c);after=FieldBuilder(c)
    intr=CameraIntrinsics(320,240,240.,240.,160.,120.)
    depth=np.ones((240,320),np.float32);pose=np.eye(4);pose[2,3]=.2
    backend=EfficientBackend(depth_calibration(intr),{'voxel_size_m':c['voxel_size']})
    backend.prepare_query(after.points)
    timings={k:[] for k in ['old_query','prepared_query','old_field','new_field']}
    fields={'old_field':before,'new_field':after}
    for i in range(65):
        pose[0,3]=-.3+i*.01
        backend.integrate_depth(depth,pose);backend.update_esdf()
        values={}
        methods={'old_query':lambda:backend.query_esdf(before.points),'prepared_query':backend.query_prepared}
        for name in (list(methods) if i%2 else list(methods)[::-1]):
            start=time.perf_counter();values[name]=methods[name]();elapsed=(time.perf_counter()-start)*1000
            if i>=5:timings[name].append(elapsed)
        np.testing.assert_array_equal(values['old_query'],values['prepared_query'])
        raw=values['prepared_query'];known=known_distances(raw,backend.esdf_unknown_distance);outputs={}
        for name in (list(fields) if i%2 else list(fields)[::-1]):
            start=time.perf_counter();outputs[name]=fields[name].update(raw,known,depth,pose,intr);elapsed=(time.perf_counter()-start)*1000
            if i>=5:timings[name].append(elapsed)
        for a,b in zip(outputs['old_field'],outputs['new_field']):np.testing.assert_array_equal(a,b)
    result=dict(passed=True,grid_cells=len(after.points),frames_compared=65,all_distances_and_sources_identical=True,
                exact_field_each_frame=True,timings={k:summarize(v) for k,v in timings.items()})
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':main()
