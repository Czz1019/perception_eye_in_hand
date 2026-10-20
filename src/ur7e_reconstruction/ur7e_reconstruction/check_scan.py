"""Offline sampled clearance check against the unchanged Gazebo scene geometry."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
import xacro,yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
root=ET.fromstring(xacro.process_file(str(Path(get_package_share_directory('ur7e_d435_gazebo'))/'urdf/ur7e_d435.urdf.xacro')).toxml())
names=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
joints={j.find('child').get('link'):j for j in root.findall('joint')}
def fk(link,q):
 if link=='base_link':return np.eye(4)
 j=joints[link];o=j.find('origin');T=np.eye(4)
 if o is not None:
  T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix();T[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ')
 if j.get('type') in ['revolute','continuous']:
  a=np.fromstring(j.find('axis').get('xyz'),sep=' ');R=np.eye(4);R[:3,:3]=Rotation.from_rotvec(a*q[names.index(j.get('name'))]).as_matrix();T=T@R
 return fk(j.find('parent').get('link'),q)@T


def origin(elem):
 T=np.eye(4)
 if elem is not None:
  T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(elem.get('rpy','0 0 0'),sep=' ')).as_matrix();T[:3,3]=np.fromstring(elem.get('xyz','0 0 0'),sep=' ')
 return T
objects=yaml.safe_load((Path(get_package_share_directory('ur7e_d435_gazebo'))/'config/collision_world_gazebo_tall.yml').read_text())['world_model']['coll_objs']
clouds={}
for link in root.findall('link'):
 name=link.get('name')
 if name in ['ground_plane','world','base_link','base_link_inertia']:continue
 pts=[]
 for collision in link.findall('collision'):
  g=collision.find('geometry');mesh=g.find('mesh');box=g.find('box');cylinder=g.find('cylinder')
  if mesh is not None:
   path=mesh.get('filename').replace('file://','')
   data=Path(path).read_bytes();v=np.frombuffer(data[84:],dtype=[('n','<f4',(3,)),('p','<f4',(3,3)),('a','<u2')])['p'].reshape(-1,3)
  elif box is not None:
   dims=np.fromstring(box.get('size'),sep=' ');v=np.array(np.meshgrid(*[np.linspace(-d/2,d/2,max(3,int(d/.008))) for d in dims])).reshape(3,-1).T
  elif cylinder is not None:
   r=float(cylinder.get('radius'));h=float(cylinder.get('length'));t,z=np.meshgrid(np.linspace(0,2*np.pi,30),np.linspace(-h/2,h/2,20));v=np.c_[r*np.cos(t.ravel()),r*np.sin(t.ravel()),z.ravel()]
  else: continue
  T=origin(collision.find('origin'));pts.append(v@T[:3,:3].T+T[:3,3])
 if pts:clouds[name]=np.unique(np.concatenate(pts),axis=0)

def clearance(q):
 best=(99,None)
 for name,v in clouds.items():
  T=fk(name,q);p=v@T[:3,:3].T+T[:3,3]
  for obstacle,obj in objects['cube'].items():
   if obstacle=='ground':continue
   if not np.allclose(obj['pose'][3:],[0,0,0,1]):raise ValueError('Expected axis-aligned walls')
   delta=np.abs(p-obj['pose'][:3])-np.asarray(obj['dims'])/2
   d=np.linalg.norm(np.maximum(delta,0),axis=1)+np.minimum(np.max(delta,axis=1),0)
   if d.min()<best[0]:best=(float(d.min()),name)
  for obj in objects['sphere'].values():
   d=np.linalg.norm(p-obj['position'],axis=1)-obj['radius']
   if d.min()<best[0]:best=(float(d.min()),name)
 return best
def main():
 import argparse,json
 parser=argparse.ArgumentParser(description='Sample actual URDF collision-mesh vertices along the fixed scan; this is not a general collision planner.')
 parser.add_argument('--views',type=Path,default=Path(get_package_share_directory('ur7e_reconstruction'))/'config/scan_views.yaml')
 parser.add_argument('--output',type=Path,required=True)
 args=parser.parse_args()
 cfg=yaml.safe_load(args.views.read_text())
 report=[]
 for a,b in zip(cfg['views'][:-1],cfg['views'][1:]):
  best=(99,None,None)
  for u in np.linspace(0,1,75):
   d,name=clearance((1-u)*np.array(a['positions'])+u*np.array(b['positions']))
   if d<best[0]:best=d,name,float(u)
  record=dict(start=a['name'],end=b['name'],minimum_sampled_clearance_m=best[0],link=best[1],fraction=best[2])
  report.append(record)
  print(record,flush=True)
 args.output.parent.mkdir(parents=True,exist_ok=True)
 args.output.write_text(json.dumps(report,indent=2))
 return 0 if all(v['minimum_sampled_clearance_m']>.005 for v in report) else 1

if __name__=='__main__':
 raise SystemExit(main())
