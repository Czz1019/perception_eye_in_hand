"""Exercise spawn-based geometry worker using saved sensor evidence."""
import argparse,time,json
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
from ur7e_gazebo_esdf_sage.config import load_mapping,identity
from ur7e_gazebo_esdf_sage.partial_map import load_snapshot
from ur7e_real_static.field import FrozenMap
from ur7e_gazebo_esdf_sage.online_v2 import init_geometry_worker,process_geometry
import yaml

def main():
 p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();out=Path(a.output);c=load_mapping(out/'mapping.yaml');path=next((out/'maps').glob('*/partial_policy.npz'));f=FrozenMap.load(path.parent/'frozen_map.npz',identity(c));base=load_snapshot(f,path)
 with np.load(path) as d:static=replace(base,distance=d['obstacle'].copy())
 extra=yaml.safe_load((out/'scan.yaml').read_text())['depth_self_mask']
 files=sorted((out/'online').glob('observation_*.npz'));samples=[files[0],files[len(files)//2],files[-1]]
 with ProcessPoolExecutor(max_workers=1,mp_context=get_context('spawn'),initializer=init_geometry_worker,initargs=(c,extra,base,static)) as pool:
  occupied=np.zeros(base.distance.shape,bool);distance=base.distance;results=[]
  for file in samples:
   with np.load(file) as d:
    k=d['intrinsics'];intr=SimpleNamespace(fx=k[0],fy=k[1],cx=k[2],cy=k[3],width=int(k[4]),height=int(k[5]));args=(d['ray_depth'],d['pose'],intr,d['q'],occupied,distance)
   start=time.monotonic();depth,occupied,hit,distance,changed,timing=pool.submit(process_geometry,*args).result(timeout=15)
   results.append(dict(file=file.name,wall_s=time.monotonic()-start,occupied=int(occupied.sum())))
 print(json.dumps(results,indent=2))
if __name__=='__main__':main()
