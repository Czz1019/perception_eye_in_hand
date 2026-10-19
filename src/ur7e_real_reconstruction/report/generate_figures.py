"""Offline report assets from the frozen 2026-09-28 scan; no ROS or robot access."""
from pathlib import Path
import json,hashlib,shutil
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

REPORT=Path(__file__).resolve().parent
ROOT=REPORT.parents[2]
SOURCE=ROOT/'log/real_reconstruction_20260928/real/scan_20260928_130419_942f86'
ASSETS=REPORT/'assets';ASSETS.mkdir(exist_ok=True)
metadata=json.loads((SOURCE/'metadata.json').read_text())
views=metadata['views']
records=[]
for view in views:
    name=f'observation_{view["id"]:02d}_rgb.png'
    source=SOURCE/view['path']/'rgb.png'
    shutil.copy2(source,ASSETS/name)
    record=dict(view)
    record['image']='assets/'+name
    record['local_time']=datetime.fromtimestamp(view['depth_stamp'],ZoneInfo('Asia/Shanghai')).isoformat(timespec='milliseconds')
    record['rgb_sha256']=hashlib.sha256(source.read_bytes()).hexdigest()
    records.append(record)
(REPORT/'observations.json').write_text(json.dumps(records,ensure_ascii=False,indent=2)+'\n')
for name in ['final_checks.json','metadata.json']:
    shutil.copy2(SOURCE/name,REPORT/name)
shutil.copy2(SOURCE/'final_rviz.png',ASSETS/'final_rviz.png')
shutil.copy2(ROOT/'src/ur7e_real_reconstruction/validation/mock_results.json',REPORT/'mock_results.json')
with np.load(SOURCE/'surface.npz',allow_pickle=False) as data:
    v,t,c=data['vertices'],data['triangles'],data['colors']
tri=v[t];face_colors=c[t].astype(float).mean(axis=1)/255.
lo,hi=v.min(0),v.max(0);mid=(lo+hi)/2;span=hi-lo
plt.rcParams.update({'font.size':10,'axes.titlesize':13,'figure.facecolor':'white'})
fig=plt.figure(figsize=(15,7.2),layout='constrained')
ax=fig.add_subplot(121,projection='3d')
mesh=Poly3DCollection(tri,facecolors=face_colors,edgecolors='none',linewidths=0)
ax.add_collection3d(mesh)
for setlim,a,b in [(ax.set_xlim,lo[0],hi[0]),(ax.set_ylim,lo[1],hi[1]),(ax.set_zlim,lo[2],hi[2])]:setlim(a-.03,b+.03)
ax.set_box_aspect(span+.06);ax.view_init(elev=48,azim=-63)
ax.set(xlabel='X [m]',ylabel='Y [m]',zlabel='Z [m]',title='Frozen colored mesh | perspective')
ax=fig.add_subplot(122)
order=np.argsort(tri[:,:,2].mean(axis=1))
ax.add_collection(PolyCollection(tri[order,:,:2],facecolors=face_colors[order],edgecolors='none'))
ax.set_xlim(lo[0]-.04,hi[0]+.04);ax.set_ylim(lo[1]-.04,hi[1]+.04);ax.set_aspect('equal')
ax.set(xlabel='X [m]',ylabel='Y [m]',title='Frozen colored mesh | top view (XY)')
ax.set_facecolor('#eef1f4');ax.grid(alpha=.15)
fig.suptitle('Real UR7e + D435i | 859 fused frames | 35 recorded views\n17,167 vertices / 24,731 triangles | base_link | TSDF voxel 0.015 m',fontsize=15)
fig.savefig(ASSETS/'final_mesh_views.png',dpi=160);plt.close(fig)
poses=np.array([a['T_base_depth'] for a in views]);p=poses[:,:3,3];z=poses[:,:3,2]
fig=plt.figure(figsize=(12,6),layout='constrained')
ax=fig.add_subplot(121,projection='3d')
ax.plot(*p.T,color='#9aa6b2',lw=.7)
sc=ax.scatter(*p.T,c=np.arange(1,36),cmap='viridis',s=30)
ax.quiver(*p.T,*z.T,length=.065,color='#e07c28',linewidth=.8)
for i in [0,4,9,14,19,24,29,34]:ax.text(*p[i],f' {i+1:02}',fontsize=8)
ax.set(xlabel='X [m]',ylabel='Y [m]',zlabel='Z [m]',title='Depth optical poses in base_link');ax.view_init(elev=25,azim=-60)
ax=fig.add_subplot(122)
ax.plot(range(1,36),p[:,0],'-o',ms=3,label='X');ax.plot(range(1,36),p[:,1],'-o',ms=3,label='Y');ax.plot(range(1,36),p[:,2],'-o',ms=3,label='Z')
ax.set(xlabel='Recorded observation ID',ylabel='Position [m]',title='Recorded camera positions');ax.grid(alpha=.25);ax.legend()
fig.suptitle('35 actual recorded views | orange arrows: optical +Z\nConnecting lines indicate record order, not the continuous robot path',fontsize=13)
fig.savefig(ASSETS/'observation_positions.png',dpi=150);plt.close(fig)
manifest={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(ASSETS.glob('*.png'))}
(REPORT/'figure_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(f'{len(records)} original RGB images copied without changes; {len(manifest)} images total')
