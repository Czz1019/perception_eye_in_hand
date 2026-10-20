# Adapted from ur7e_auto_reconstruction; RTDE transport is shared, not copied.
"""Reviewable offline artifacts; never imports an RTDE control interface."""
import argparse
import json
from pathlib import Path
import numpy as np
from .trajectory import load_inputs,make_plan


def render(plan,directory,geometry=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig=plt.figure(figsize=(11,7))
    ax=fig.add_subplot(111,projection='3d')
    for i,segment in enumerate(plan['segments']):
        xyz=np.array(segment['camera_path_m'])
        ax.plot(*xyz.T,color='orange' if i==0 else 'steelblue')
        ax.scatter(*xyz[-1],c='red')
        ax.text(*xyz[-1],segment['to'])
    ax.set(xlabel='base_link X (m)',ylabel='Y (m)',zlabel='Z (m)',
           title='Camera path: orange = entry; no collision validation')
    ax.set_box_aspect([1,1,.7]);fig.tight_layout();fig.savefig(directory/'route.png',dpi=150);plt.close(fig)
    lines=['# 自动扫描轨迹预览','',f"数据来源：`{plan['source']}`。SHA256：`{plan['sha256']}`。",'',
           '这是一份关节轨迹预览，不是无碰撞证明。场景避障和完整自身碰撞检查未启用。',
           '旧选点报告中相机/手腕球体间距未通过的记录仍然有效；本预览没有消除这个未验证项。',
           '实际开始前将核对当前关节角、模型、参数和文件完整性。', '',
           f"初始关节角（度）：{np.round(np.rad2deg(plan['q_start']),2).tolist()}",
           f"标称总运动时间：{plan['nominal_motion_sec']:.1f} 秒；不含停稳、采集、GPU 初始化/导出和速度缩放。",'',
           '| 段终点 | 标称时间 s | 最大关节变化 ° | 相机最低高度 m |','|---|---:|---:|---:|']
    for s in plan['segments']:
        lines.append(f"| {s['to']} | {s['duration_sec']:.2f} | {s['max_delta_deg']:.2f} | {s['camera_min_z_m']:.3f} |")
    lines+=['','![相机路径](route.png)','','首段是当前姿态到 S01 的关节插值；相机路径通常不是竖直直线。',
            '若首段不适合现场，请先手动移至合适的高位起点，再重新 prepare。不要执行旧报告中被拒绝的垂直提升路径。']
    if geometry is not None:
        count=len(plan['segments'])
        fig=plt.figure(figsize=(14,4*((count+3)//4)))
        links=['base_link','shoulder_link','upper_arm_link','forearm_link',
               'wrist_1_link','wrist_2_link','wrist_3_link','tool0','camera_depth_optical_frame']
        for i,segment in enumerate(plan['segments']):
            ax=fig.add_subplot((count+3)//4,4,i+1,projection='3d')
            poses=geometry.poses_batch(np.linspace(segment['q_start'],segment['q_end'],3))
            for j,color in enumerate(['gray','orange','steelblue']):
                xyz=np.array([poses[link][j,:3,3] for link in links])
                ax.plot(*xyz.T,'o-',color=color,markersize=2,label=['start','middle','end'][j])
            ax.set(title=('Entry -> ' if i==0 else plan['segments'][i-1]['to']+' -> ')+segment['to'],
                   xlim=(-.6,.9),ylim=(-.8,.6),zlim=(0,1.2),xlabel='X',ylabel='Y',zlabel='Z')
            ax.set_box_aspect((1.5,1.4,1.2));ax.view_init(25,-60)
            if i==0:ax.legend(fontsize=7)
        fig.suptitle('Arm link origins in base_link (m): start / middle / end; no collision geometry')
        fig.tight_layout();fig.savefig(directory/'arm_segments.png',dpi=140,bbox_inches='tight',pad_inches=.25);plt.close(fig)
        lines+=['','## 各段机械臂形态预览','',
                '灰色为起点、橙色为关节插值中点、蓝色为终点；图中是连杆原点连线，不是实体碰撞外形。',
                '![各段机械臂形态](arm_segments.png)']
    (directory/'PREVIEW.md').write_text('\n'.join(lines)+'\n')


def main():
    p=argparse.ArgumentParser()
    for name in ['mapping','control','waypoints','q','output']:p.add_argument('--'+name,required=True)
    args=p.parse_args()
    mapping,control,route,geometry=load_inputs(args.mapping,args.control,args.waypoints)
    plan=make_plan(json.loads(args.q),control,route,geometry,source='offline_reference_NOT_EXECUTABLE')
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    (out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n');render(plan,out,geometry)
    print(json.dumps({'path':str(out),'motion_sec':plan['nominal_motion_sec']}))


if __name__=='__main__':main()
