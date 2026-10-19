"""Explicit bounded-unknown planning policy, separate from environmental evidence.

Positive feasibility distance is NOT a claim of observed free space. Unknown
voxels remain source=0. This simulation policy accepts a bounded sensing risk.
"""
from dataclasses import replace
from pathlib import Path
import hashlib,json
import numpy as np
from scipy.ndimage import distance_transform_edt
from ur7e_nvblox.depth import known_distances

POLICY='bounded_unknown_v1'
BAND=.08  # Experimental exploration limit: four 20 mm grid cells, not a noise bound.
WEIGHT=40.


def fields(frozen,owned,raw,known,band=BAND):
    snap=frozen.snapshot;shape=snap.distance.shape;res=snap.resolution
    occupied=np.asarray(known)&(np.asarray(raw)<=.02)
    observed=(frozen.source>0)&~occupied
    if not observed.any() or not occupied.any():raise ValueError('Partial planning requires both surface and free-space evidence')
    # Proximity only to actual environmental evidence, never a robot sweep or
    # body-owned voxel. Ownership itself is not dilated.
    unknown_depth=distance_transform_edt(~observed.reshape(shape))*res
    admitted=(unknown_depth.reshape(-1)<=band+1e-9)|observed|owned
    admitted&=~occupied
    obstacle=distance_transform_edt(np.pad((~occupied).reshape(shape),1))[1:-1,1:-1,1:-1]*res-np.sqrt(3)*res/2
    obstacle=np.minimum(obstacle.reshape(-1),np.where(known,raw,np.inf)).reshape(shape)
    domain=distance_transform_edt(np.pad(admitted.reshape(shape),1))[1:-1,1:-1,1:-1]*res-np.sqrt(3)*res/2
    feasibility=np.minimum(obstacle,domain).astype(np.float32)
    # Do not penalize actual body interiors as uncertain environment.
    uncertainty=unknown_depth.astype(np.float32);uncertainty.reshape(-1)[observed|owned]=0.
    return feasibility,obstacle.astype(np.float32),uncertainty,admitted


def prepare(frozen,session,band=BAND):
    session=Path(session)
    with np.load(session/'robot_ownership.npz') as d:owned=d['owned'].copy()
    with np.load(session/'native_esdf.npz') as d:raw=d['distance'].copy();known=known_distances(raw,float(d['unknown']))
    distance,obstacle,uncertainty,admitted=fields(frozen,owned,raw,known,band)
    metadata=dict(policy=POLICY,environment_map_uid=frozen.uid,unknown_band_m=band,unknown_cost_weight=WEIGHT,
                  unknown_is_free=False,environment_source_changed=False,fixture_truth_used=False,
                  ownership_dilated=False,admitted_unknown_cells=int(np.count_nonzero(admitted&(frozen.source==0)&~owned)))
    h=hashlib.sha256(json.dumps(metadata,sort_keys=True).encode())
    for arr in [distance,obstacle,uncertainty,admitted]:h.update(arr.tobytes())
    metadata['policy_uid']=h.hexdigest()
    np.savez_compressed(session/'partial_policy.npz',distance=distance,obstacle=obstacle,uncertainty=uncertainty,admitted=admitted,metadata=json.dumps(metadata,sort_keys=True))
    (session/'partial_policy.json').write_text(json.dumps(metadata,indent=2))
    distance.flags.writeable=False
    return replace(frozen.snapshot,distance=distance),metadata


def load_snapshot(frozen,path):
    with np.load(path) as d:
        m=json.loads(str(d['metadata']));arrays=[d[k].copy() for k in ['distance','obstacle','uncertainty','admitted']]
    uid=m.pop('policy_uid');h=hashlib.sha256(json.dumps(m,sort_keys=True).encode())
    for arr in arrays:h.update(arr.tobytes())
    if h.hexdigest()!=uid or m['environment_map_uid']!=frozen.uid or m['policy']!=POLICY:raise ValueError('Partial policy identity/checksum mismatch')
    if any(x.shape!=frozen.snapshot.distance.shape for x in arrays[:3]) or arrays[3].shape!=frozen.source.shape:raise ValueError('Partial policy shape mismatch')
    arrays[0].flags.writeable=False
    return replace(frozen.snapshot,distance=arrays[0])


def attach_cost(rollout,session):
    """Add uncertainty ranking to the unchanged SAGE optimizer/collision costs."""
    import torch
    import torch.nn.functional as F
    from types import MethodType
    with np.load(Path(session)/'partial_policy.npz') as d:
        uncertainty=torch.tensor(d['uncertainty'],**rollout.tensor_args)[None,None]
    original=rollout.cost_fn
    def cost(self,state_dict,action_batch,no_coll=False,horizon_cost=True,return_dist=False):
        result=original(state_dict,action_batch,no_coll,horizon_cost,return_dist)
        if no_coll:return result
        rotation=state_dict['link_rot_seq'][...,self.link_ids,:,:];position=state_dict['link_pos_seq'][...,self.link_ids,:]
        centers=(rotation@self.local[...,None]).squeeze(-1)+position
        # Sample center and six extremities; the independent hard check covers
        # complete swept spheres, so these samples only rank uncertainty.
        offsets=torch.cat([torch.zeros((1,3),**self.tensor_args),torch.eye(3,**self.tensor_args),-torch.eye(3,**self.tensor_args)])
        points=centers[...,None,:]+self.radii[:,None,None]*offsets
        coordinates=2*(points-self.origin)/self.extent-1
        values=F.grid_sample(uncertainty,coordinates.reshape(1,1,1,-1,3),mode='bilinear',padding_mode='border',align_corners=True).reshape(points.shape[:-1])
        penalty=WEIGHT*(values/BAND).square().mean(dim=(-1,-2))
        return (result[0]+penalty,*result[1:]) if return_dist else result+penalty
    rollout.cost_fn=MethodType(cost,rollout)
