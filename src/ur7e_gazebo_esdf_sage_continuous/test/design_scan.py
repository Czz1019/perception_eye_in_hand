"""Offline fixture commissioning only. Runtime planning never imports truth."""
from pathlib import Path
import numpy as np
import yaml
from ur7e_real_static.model import Geometry
from ur7e_perception_planning_core.geometry import JOINTS
from ur7e_gazebo_esdf_sage_continuous.field import scan_samples

class Fixture:
    def query(self, p):
        p = np.asarray(p)
        distances = [p[..., 2], np.linalg.norm(p-[.65,0,.1], axis=-1)-.1]
        for center in [[.50,.20,.23],[.50,-.28,.23]]:
            d = abs(p-center)-[.12,.05,.23]
            distances.append(np.linalg.norm(np.maximum(d,0),axis=-1)+np.minimum(d.max(-1),0))
        return np.min(distances,axis=0)

def main():
    ws = Path(__file__).resolve().parents[3]
    directory = ws/'src/ur7e_gazebo_esdf_sage_continuous/config'
    g = Geometry(directory/'real_calibrated.urdf', directory/'collision_spheres.yaml')
    truth = Fixture()
    seed=np.array([-1.43534,-1.34076,.032089,-.648888,-2.64401,-1.141411])
    views=[]
    for shoulder, bases in [(0.,[-1.75,-1.15,-.55,.05,.65]),(.15,[.65,.05,-.55,-1.15,-1.75]),(.3,[-1.75,-1.15,-.55,.05,.65]),(0.,[-1.75])]:
        for base in bases:
            q=seed.copy();q[0]=base;q[1]+=shoulder;q[3]-=shoulder;views.append(q)
    qs=scan_samples(np.array(views),.002)
    centers=g.spheres_batch(qs)
    gap=float(g.self_clearance_batch(centers).min())
    environment=float((truth.query(centers)-g.radii).min())
    result=dict(min_self_gap=gap,min_environment_gap=environment,samples=len(qs))
    print(result)
    # .002 rad sampling + motion bound certifies the intervening sweep as well.
    sampling_padding=float(g.motion_bound*np.max(np.sum(abs(np.diff(qs,axis=0)),axis=1))/2)
    if gap<.019 or environment<.083+sampling_padding:raise RuntimeError('Scan transition rejected')
    (directory/'scan.yaml').write_text(yaml.safe_dump(dict(views=[q.tolist() for q in views],
        prior_clearance=.083,capture_frames=5,max_scan_speed=.25,scan_acceleration=.4,
        default_goal=[.50,0,.60],rolling_speed=.15,control_dt=.1,wall_feedback_timeout=2.,
        depth_self_mask=dict(centers=[[0.,0.,.037706]],radii=[.085],description="Fixed UR base.stl enclosing sphere, base_link frame; only invalidates pixels."),
        fixture_only=True,prior_description='Scan motion only: fixed commissioned corridor; never copied into the planning field.',
        commissioning=dict(**result,sampling_padding=sampling_padding)),sort_keys=False))
    (directory/'initial_positions.yaml').write_text(yaml.safe_dump(dict(zip(JOINTS,views[0].tolist()))))

if __name__=='__main__':main()
