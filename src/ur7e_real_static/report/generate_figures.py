"""Offline report figures from saved data only. Never publishes ROS commands."""
from pathlib import Path
import json,xml.etree.ElementTree as ET,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation
import trimesh
from ament_index_python.packages import get_package_share_directory
from ur7e_real_static.config import load_config
from ur7e_real_static.model import Geometry

REPORT=Path(__file__).resolve().parent;ROOT=REPORT.parents[2];RUN=ROOT/'log/real_first_scan_20260927';ASSETS=REPORT/'assets'
ASSETS.mkdir(exist_ok=True)
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':10,'figure.facecolor':'#f6f8fb','axes.facecolor':'#f6f8fb','savefig.facecolor':'#f6f8fb'})
c=load_config('/home/wqj/ur7e_real_setup/preview.yaml');g=Geometry(c['robot_urdf'],c['collision_spheres'])
with np.load(RUN/'output/frozen_map.npz',allow_pickle=False) as a:meta=json.loads(str(a['metadata']));uid=str(a['uid'])
qs=np.array(meta['views']);assert qs.shape==(21,6)
with np.load(RUN/'output/frozen_surface_pointcloud.npz',allow_pickle=False) as a:
    cloudmeta=json.loads(str(a['metadata']));raw=a['data'].tobytes()
dtype=np.dtype({'names':['x','y','z','rgb','distance'],'formats':['<f4','<f4','<f4','<u4','<f4'],'offsets':[0,4,8,12,16],'itemsize':20})
pc=np.frombuffer(raw,dtype=dtype);xyz=np.column_stack([pc[k] for k in ['x','y','z']]);rgb=np.column_stack([(pc['rgb']>>16)&255,(pc['rgb']>>8)&255,pc['rgb']&255])/255
assert len(pc)==cloudmeta['width']*cloudmeta['height']
xml=ET.parse(c['robot_urdf']).getroot();meshes=[]
for link in xml.findall('link'):
    for el in link.findall('collision'):
        m=el.find('geometry/mesh')
        if m is None:continue
        uri=m.get('filename');package,rel=uri[len('package://'):].split('/',1);path=Path(get_package_share_directory(package))/rel
        mesh=trimesh.load(path,force='mesh');tri=np.asarray(mesh.triangles)
        o=el.find('origin');t=np.eye(4)
        if o is not None:t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix();t[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ')
        meshes.append((link.get('name'),tri,t))

def transform(points,t):return points@t[:3,:3].T+t[:3,3]
def mesh_plot(ax,tri,t,color):
    points=transform(tri,t);norm=np.cross(points[:,1]-points[:,0],points[:,2]-points[:,0]);norm/=np.maximum(np.linalg.norm(norm,axis=1,keepdims=True),1e-12)
    light=np.array([.2,-.5,.8]);light/=np.linalg.norm(light);shade=.5+.5*np.abs(norm@light);colors=np.asarray(matplotlib.colors.to_rgb(color))[None,:]*shade[:,None]
    ax.add_collection3d(Poly3DCollection(points,facecolors=colors,edgecolors='none',zsort='average'))
def setup(ax,wide=False):
    ax.view_init(elev=26,azim=38)
    ax.set(xlim=(-.85,1.05) if wide else (-.25,.85),ylim=(-1.05,.8) if wide else (-.95,.25),zlim=(-.1,1.05),xlabel='X / m',ylabel='Y / m',zlabel='Z / m')
    ax.set_box_aspect((1.9,1.85,1.15) if wide else (1.1,1.2,1.15));ax.tick_params(labelsize=8,pad=0)
    for axis in (ax.xaxis,ax.yaxis,ax.zaxis):axis.pane.fill=False
    ax.grid(True,alpha=.2)
def robot(ax,poses):
    for link,tri,t in meshes:mesh_plot(ax,tri,poses[link]@t,'#2189bd' if 'wrist' not in link else '#397696')
    camera=trimesh.creation.box(extents=[.02505,.09,.025]);ct=poses['camera_link'].copy();ct[:3,3]=transform(np.array([0.,-.0175,0.]),ct);mesh_plot(ax,np.asarray(camera.triangles),ct,'#ecaa38')
def frustum(ax,t):
    info=meta['camera_info'];fx,fy=info['k'][0],info['k'][4];cx,cy=info['k'][2],info['k'][5];w,h=info['width'],info['height'];z=.28
    corners=np.array([[(u-cx)/fx*z,(v-cy)/fy*z,z] for u,v in [(0,0),(w,0),(w,h),(0,h)]])
    corners=transform(corners,t);o=t[:3,3]
    for p in corners:ax.plot(*np.array([o,p]).T,color='#d08814',lw=1.3,alpha=.8)
    loop=np.vstack((corners,corners[0]));ax.plot(*loop.T,color='#d08814',lw=1.3)
    ax.quiver(*o,*(t[:3,2]*.36),color='#cf4b1e',arrow_length_ratio=.15,lw=2)
    ax.scatter(*o,c='#cf4b1e',s=35,depthshade=False)

observations=[]
for i,q in enumerate(qs,1):
    poses=g.fk(q);t=poses[g.camera];p=t[:3,3];rpy=Rotation.from_matrix(t[:3,:3]).as_euler('xyz',degrees=True)
    observations.append(dict(id=i,q_rad=q.tolist(),q_deg=np.rad2deg(q).tolist(),camera_xyz_m=p.tolist(),camera_optical_z=t[:3,2].tolist(),camera_rpy_deg=rpy.tolist(),image=f'assets/observation_{i:02d}.png'))
    fig=plt.figure(figsize=(12,7.6));ax=fig.add_axes([.0,.10,.71,.8],projection='3d');setup(ax)
    inside=(xyz[:,0]>=-.25)&(xyz[:,0]<=.85)&(xyz[:,1]>=-.95)&(xyz[:,1]<=.25)
    back=xyz[inside][::2];ax.scatter(*back.T,c='#b2bdc7',s=1,alpha=.28,depthshade=False)
    robot(ax,poses);frustum(ax,t)
    fig.text(.045,.945,f'观察姿态 {i:02d} / 21',size=22,weight='bold',color='#173a54')
    fig.text(.045,.905,'真实关节记录 + 校准 URDF 重绘  |  非现场照片、非该姿态的单帧深度图',size=10,color='#526575')
    info=fig.add_axes([.735,.12,.25,.73]);info.axis('off')
    info.text(0,.96,'深度相机位姿',size=15,weight='bold',color='#173a54')
    info.text(0,.79,f'base_link 下的位置 / m\nX  {p[0]: .4f}\nY  {p[1]: .4f}\nZ  {p[2]: .4f}',linespacing=1.65,size=12)
    info.text(0,.58,'光学 Z 轴为观察方向\n橙色线框：理论相机视锥\n视锥绘制长度：0.28 m',size=11,linespacing=1.7)
    info.text(0,.29,'六关节角度 / °\n'+ '\n'.join(f'J{k+1}  {v:8.2f}' for k,v in enumerate(np.rad2deg(q))),size=11,linespacing=1.5)
    fig.text(.045,.045,'灰色背景为最终融合地图，仅作空间参照；不表示该观察姿态已看到背景中的全部区域。',size=10,color='#526575')
    fig.savefig(ASSETS/f'observation_{i:02d}.png',dpi=135);plt.close(fig)
    print(f'Pose {i:02d} rendered',flush=True)
(REPORT/'observations.json').write_text(json.dumps(observations,ensure_ascii=False,indent=2))

# Final map, directly from archived PointCloud2, with original distance colors.
fig=plt.figure(figsize=(14,7));left=fig.add_subplot(121,projection='3d');setup(left,wide=True);left.scatter(*xyz.T,c=rgb,s=3,depthshade=False);left.set_xlim(xyz[:,0].min()-.05,xyz[:,0].max()+.05);left.set_ylim(xyz[:,1].min()-.05,xyz[:,1].max()+.05);left.set_zlim(xyz[:,2].min()-.03,xyz[:,2].max()+.08);left.set_box_aspect((np.ptp(xyz[:,0])+.1,np.ptp(xyz[:,1])+.1,np.ptp(xyz[:,2])+.11));left.set_title('最终近表面 ESDF · 三维视图',pad=16,size=16)
right=fig.add_subplot(122);right.set_facecolor('#f6f8fb');right.scatter(xyz[:,0],xyz[:,1],c=rgb,s=3);right.set_aspect('equal');right.set(xlabel='X / m',ylabel='Y / m',title='俯视投影（包含不同高度的点）');right.grid(alpha=.2)
fig.suptitle('最终地图结果｜1233 帧 · 21 个观察姿态 · 2.5 cm',size=21,weight='bold',color='#173a54',y=.965)
fig.text(.5,.045,'直接读取冻结时保存的 12036 个近表面点；保留原始距离着色，颜色不是物体 RGB。',ha='center',size=11,color='#526575')
fig.subplots_adjust(bottom=.13,top=.87,wspace=.13);fig.savefig(ASSETS/'final_map_data.png',dpi=150);plt.close(fig)

# 21 pose positions and directions, no interpolation of the unknown path.
fig=plt.figure(figsize=(11,7.5));ax=fig.add_subplot(111,projection='3d')
for item in observations:
    p=np.array(item['camera_xyz_m']);z=np.array(item['camera_optical_z']);ax.scatter(*p,c='#227fad',s=22);ax.text(*p,f" {item['id']:02d}",fontsize=8);ax.quiver(*p,*(z*.06),color='#cd7325',lw=.9,arrow_length_ratio=.18)
ax.view_init(24,-62);ax.set(xlabel='X / m',ylabel='Y / m',zlabel='Z / m');ax.set_box_aspect((1,.6,.7));ax.set_title('21 个深度相机观察位置与方向（离散记录）',size=18,pad=24)
fig.text(.5,.035,'序号按视角首次纳入记录的顺序；箭头仅表示观察方向，不连接或推测机械臂的实际运动路径。',ha='center',size=10)
fig.savefig(ASSETS/'observation_positions.png',dpi=150);plt.close(fig)
manifest={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in ASSETS.glob('*.png')}
(REPORT/'figure_manifest.json').write_text(json.dumps(manifest,indent=2));print('All data figures completed.',flush=True)
