"""Conservative static ESDF; immutable, checksummed persistence without pickle."""
from dataclasses import dataclass
from pathlib import Path
import hashlib
import json
import os
import time
import numpy as np
from scipy.ndimage import distance_transform_edt
from ur7e_perception_planning_core.grid import Snapshot, grid_points, observed_free


def digest(metadata, arrays):
    h=hashlib.sha256(json.dumps(metadata,sort_keys=True,allow_nan=False).encode())
    for a in arrays: h.update(np.ascontiguousarray(a).tobytes())
    return h.hexdigest()


@dataclass(frozen=True)
class FrozenMap:
    snapshot: Snapshot
    source: np.ndarray
    metadata: dict
    uid: str

    @classmethod
    def create(cls, distance, source, origin, dims, resolution, metadata):
        dims=np.array(dims,dtype=np.int64,copy=True);origin=np.array(origin,dtype=np.float64,copy=True)
        d=np.asarray(distance,dtype=np.float32).reshape(-1).copy();s=np.asarray(source,dtype=np.uint8).reshape(-1).copy()
        if (dims.shape!=(3,) or origin.shape!=(3,) or np.any(dims<2) or np.prod(dims)>2_000_000
            or d.size!=np.prod(dims) or s.size!=d.size or not np.isfinite(d).all()
            or not np.isfinite(origin).all() or not np.isfinite(resolution) or resolution<=0 or np.any(s>2)):
            raise ValueError('Invalid frozen map')
        if np.any(d[s==0]>0): raise ValueError('Unknown cells must never have positive distance')
        meta=dict(metadata,origin=origin.tolist(),dimensions=dims.tolist(),resolution=float(resolution),format=1)
        uid=digest(meta,[d,s]);d.flags.writeable=False;s.flags.writeable=False
        dims.flags.writeable=False;origin.flags.writeable=False
        snapshot=Snapshot(origin,float(resolution),dims,d.reshape(tuple(dims[::-1])),
                          float(meta['stamp']),int(meta['epoch']),int(meta['sequence']),time.monotonic())
        return cls(snapshot,s,meta,uid)

    def save(self,path):
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        temporary=path.with_suffix('.tmp')
        with temporary.open('wb') as f:
            np.savez_compressed(f,distance=self.snapshot.distance,source=self.source,
                                metadata=json.dumps(self.metadata,sort_keys=True),uid=self.uid)
        os.replace(temporary,path)

    @classmethod
    def load(cls,path,expected_identity):
        with np.load(path,allow_pickle=False) as data:
            meta=json.loads(str(data['metadata']))
            if meta.get('format')!=1 or meta.get('identity')!=expected_identity:
                raise ValueError('Map model/calibration/configuration identity mismatch')
            result=cls.create(data['distance'],data['source'],meta['origin'],meta['dimensions'],meta['resolution'],meta)
            if result.uid!=str(data['uid']): raise ValueError('Map checksum mismatch')
        return result


class FieldBuilder:
    def __init__(self,c):
        self.c=c
        self.origin,self.dims,self.points=grid_points(c['grid_center'],c['grid_size'],c['grid_resolution'])
        self.source=np.zeros(len(self.points),dtype=np.uint8)
        for box in c['commissioned_free_boxes']:
            self.source[np.all((self.points>=box['min'])&(self.points<=box['max']),axis=1)]=2
        self.cached_free=None;self.cached_distance=None;self.recomputed=False
        self.edt=distance_transform_edt
        if c.get('free_space_backend','cpu')=='cuda':
            from ur7e_perception_planning_core.edt import CudaEDT
            self.edt=CudaEDT()

    def update(self,raw,known,depth,pose,intr):
        padding=.05+np.sqrt(3)*self.c['grid_resolution']/2
        self.source[observed_free(self.points,depth,pose,intr,padding)]=1
        free=self.source>0
        free[known & (raw<=.02)]=False
        self.recomputed=self.cached_free is None or not np.array_equal(free,self.cached_free)
        if self.recomputed:
            volume=free.reshape(tuple(self.dims[::-1]))
            # Crop only the zero exterior; retain a full unknown-cell halo.
            # No support plane or simulation priors are introduced here.
            distance=np.zeros(volume.shape,dtype=np.float64)
            if np.any(volume):
                bounds=[]
                for axis in range(3):
                    ids=np.flatnonzero(np.any(volume,axis=tuple(i for i in range(3) if i!=axis)))
                    bounds.append(slice(ids[0],ids[-1]+1))
                region=tuple(bounds)
                distance[region]=self.edt(np.pad(volume[region],1))[1:-1,1:-1,1:-1]*self.c['grid_resolution']
            self.cached_distance=distance.reshape(-1)
            self.cached_free=free.copy()
        d=self.cached_distance-np.sqrt(3)*self.c['grid_resolution']/2
        d=np.minimum(d,np.where(known,raw,np.inf))
        d[~free]=np.minimum(d[~free],0.)
        source=self.source.copy();source[~free]=0
        return d.astype(np.float32),source
