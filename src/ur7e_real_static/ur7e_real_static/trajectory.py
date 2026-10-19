"""Dense whole-arm checks and stop-at-waypoint quintic time parameterization."""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation


def edge_clear(g, snapshot, a, b, margin=.03, self_margin=.015):
    a,b=np.asarray(a),np.asarray(b)
    if a.shape!=(6,) or b.shape!=(6,) or not np.isfinite([a,b]).all(): return False
    if np.any(np.minimum(a,b)<g.lower) or np.any(np.maximum(a,b)>g.upper): return False
    motion=g.motion_bound*float(np.sum(abs(b-a)))
    count=max(2,int(np.ceil(motion/.004))+1)
    if count>100000: return False
    padding=motion/(2*(count-1))
    for start in range(0,count,256):
        alpha=np.arange(start,min(count,start+256))/(count-1)
        centers=g.spheres_batch(a+(b-a)*alpha[:,None])
        if np.min(snapshot.query(centers)-g.radii)<margin+padding: return False
        if np.min(g.self_clearance_batch(centers))<self_margin+2*padding: return False
    return True


def simplify(g,snapshot,points,margin,self_margin):
    result=[np.array(points[0])];i=0
    while i<len(points)-1:
        j=len(points)-1
        while j>i+1 and not edge_clear(g,snapshot,points[i],points[j],margin,self_margin): j-=1
        if not edge_clear(g,snapshot,points[i],points[j],margin,self_margin): raise ValueError('Unsafe trajectory segment')
        result.append(np.asarray(points[j]));i=j
    return np.asarray(result)


@dataclass(frozen=True)
class Trajectory:
    positions: np.ndarray
    times: np.ndarray
    map_uid: str
    identity: str
    goal: np.ndarray
    sage_iterations: int

    def sample(self,t):
        i=int(np.clip(np.searchsorted(self.times,t,side='right')-1,0,len(self.times)-2))
        duration=self.times[i+1]-self.times[i];u=float(np.clip((t-self.times[i])/duration,0,1))
        h=10*u**3-15*u**4+6*u**5
        return self.positions[i]+h*(self.positions[i+1]-self.positions[i])


def parameterize(points,speed,acceleration,map_uid,identity,goal,iterations=0):
    q=np.asarray(points,dtype=float)
    if q.ndim!=2 or q.shape[1]!=6 or len(q)<2 or not np.isfinite(q).all(): raise ValueError('Invalid trajectory')
    if speed<=0 or acceleration<=0: raise ValueError('Invalid motion limits')
    delta=np.max(abs(np.diff(q,axis=0)),axis=1)
    # max derivative of 10u^3-15u^4+6u^5: 1.875; max second derivative: 10/sqrt(3).
    duration=np.maximum(.2,np.maximum(1.875*delta/speed,np.sqrt((10/np.sqrt(3))*delta/acceleration)))
    q=q.copy();q.flags.writeable=False
    return Trajectory(q,np.r_[0.,np.cumsum(duration)],map_uid,identity,np.array(goal),iterations)


def validate(g,frozen,trajectory,c):
    if trajectory.map_uid!=frozen.uid or trajectory.identity!=frozen.metadata['identity']:
        raise ValueError('Trajectory belongs to a different map/model')
    if len(trajectory.positions)!=len(trajectory.times) or trajectory.times[0]!=0 or not np.isfinite(trajectory.times).all():
        raise ValueError('Invalid trajectory times')
    dt=np.diff(trajectory.times);delta=np.max(abs(np.diff(trajectory.positions,axis=0)),axis=1)
    if np.any(dt<=0) or np.any(1.875*delta/dt>c['max_joint_speed']+1e-9) or np.any((10/np.sqrt(3))*delta/dt**2>c['max_joint_acceleration']+1e-9):
        raise ValueError('Trajectory exceeds speed/acceleration bounds')
    if not all(edge_clear(g,frozen.snapshot,a,b,c['collision_margin'],c['self_margin']) for a,b in zip(trajectory.positions,trajectory.positions[1:])):
        raise ValueError('Trajectory intersects obstacle, unknown space or self collision')
    end=g.fk(trajectory.positions[-1])[g.tool][:3,3]
    if np.linalg.norm(end-trajectory.goal)>.012: raise ValueError('Goal error exceeds 12 mm')


def model_matches(g,q,poses,position_tolerance=.003,angle_tolerance=.01):
    expected=g.fk(q)
    for link,actual in poses.items():
        if np.linalg.norm(expected[link][:3,3]-actual[:3,3])>position_tolerance: return False
        if Rotation.from_matrix(expected[link][:3,:3].T@actual[:3,:3]).magnitude()>angle_tolerance: return False
    return True
