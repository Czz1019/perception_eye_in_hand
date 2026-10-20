"""Bounded ESDF guide search; reserve time for connections after goal IK.

Adapted from ur7e_real_static.planning; fixture truth is never used here.
The guide is executed only through rolling SAGE and independent edge checks.
"""
import time
import numpy as np
from scipy.optimize import least_squares
from ur7e_real_static.trajectory import edge_clear

def route(g,snapshot,start,xyz,anchors,c,cancel=lambda:False):
    begun=time.monotonic();deadline=begun+min(30.,c['max_plan_seconds'])
    ik_deadline=begun+min(8.,.4*c['max_plan_seconds'])
    anchors=sorted(anchors,key=lambda q:np.linalg.norm(g.fk(np.asarray(q))[g.tool][:3,3]-xyz)+.02*np.linalg.norm(np.asarray(q)-start))
    def clear(a,b): return edge_clear(g,snapshot,a,b,c['collision_margin']+.005,c['self_margin'])
    def residual(q):
        centers=g.spheres(q)
        return np.r_[25*(g.fk(q)[g.tool][:3,3]-xyz),.01*(q-start),
                     5*np.maximum(0,c['collision_margin']+.015-(snapshot.query(centers)-g.radii)),
                     5*max(0,c['self_margin']+.005-g.self_clearance(centers))]
    candidates=[]
    seeds=[start,*[np.asarray(a) for a in anchors]]
    for sign in [-1,1]: seeds.append(start+np.array([0,0,sign*.7,-sign*.4,0,0]))
    for seed in seeds[:14]:
        if cancel(): raise InterruptedError('Planning canceled')
        if time.monotonic()>ik_deadline and candidates: break
        if time.monotonic()>deadline: break
        q=least_squares(residual,np.clip(seed,g.lower+.001,g.upper-.001),bounds=(g.lower,g.upper),max_nfev=80,diff_step=1e-3).x
        if np.linalg.norm(g.fk(q)[g.tool][:3,3]-xyz)<.006 and clear(q,q):
            candidates.append(q)
            if clear(start,q): return [start,q]
            if len(candidates)>=3:break
    if not candidates: raise ValueError('No observed collision-free goal IK; collect more views or change target')
    vertices=[start,*[np.asarray(a) for a in anchors if clear(a,a)],*candidates]
    goal_indices=set(range(len(vertices)-len(candidates),len(vertices)))
    queue=[0];parent={0:None}
    while queue and time.monotonic()<deadline:
        if cancel(): raise InterruptedError('Planning canceled')
        i=queue.pop(0)
        for j in sorted(set(range(len(vertices)))-set(parent),key=lambda k:np.linalg.norm(vertices[k]-vertices[i])):
            if clear(vertices[i],vertices[j]):
                parent[j]=i
                if j in goal_indices:
                    path=[]
                    while j is not None: path.append(vertices[j]);j=parent[j]
                    return path[::-1]
                queue.append(j)
    # Sampling never declares unknown cells free. All tree edges are checked.
    rng=np.random.default_rng(41);points=[start];parents=[-1]
    for _ in range(1800):
        if cancel(): raise InterruptedError('Planning canceled')
        if time.monotonic()>deadline: break
        goal=candidates[int(rng.integers(len(candidates)))]
        low=np.maximum(g.lower,np.minimum(start,goal)-.8);high=np.minimum(g.upper,np.maximum(start,goal)+.8)
        sample=goal if rng.random()<.25 else rng.uniform(low,high)
        i=int(np.argmin(np.linalg.norm(np.asarray(points)-sample,axis=1)))
        delta=sample-points[i];q=points[i]+delta*min(1.,.2/max(np.linalg.norm(delta),1e-9))
        if not clear(points[i],q): continue
        points.append(q);parents.append(i)
        if clear(q,goal):
            path=[goal];j=len(points)-1
            while j>=0:path.append(points[j]);j=parents[j]
            return path[::-1]
    raise ValueError('No checked route in observed space within search budget')



def shortcut(g,snapshot,path,c):
    """Remove detours only after checking the complete replacement edge."""
    result=[path[0]];i=0
    while i<len(path)-1:
        j=len(path)-1
        while j>i+1 and not edge_clear(g,snapshot,path[i],path[j],c['collision_margin']+.005,c['self_margin']):j-=1
        result.append(path[j]);i=j
    return result
