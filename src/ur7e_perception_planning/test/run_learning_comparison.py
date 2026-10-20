"""Headless cold-start A->B/wall benchmark, optionally importing an untouched source snapshot.

Source ROS and the workspace, use whole_control Python. Isolated ROS/Gazebo ports
must be supplied by the caller. No real-robot interfaces are used.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml
import rclpy
from geometry_msgs.msg import PoseStamped
from run_benchmark import wall_detour_check


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',required=True)
    p.add_argument('--source-root',help='Directory containing backed-up ur7e_nvblox and ur7e_perception_planning')
    p.add_argument('--esdf-hz',type=float,default=5.)
    p.add_argument('--duration',type=float,default=180.)
    args=p.parse_args()
    root=Path(__file__).resolve().parents[3];out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
    env=os.environ.copy();env['PYTHONDONTWRITEBYTECODE']='1';env['GAZEBO_MODEL_DATABASE_URI']=''
    if args.source_root:
        source=Path(args.source_root).resolve()
        env['PYTHONPATH']=os.pathsep.join([str(source/'ur7e_nvblox'),str(source/'ur7e_perception_planning'),env.get('PYTHONPATH','')])
    config=yaml.safe_load((root/'src/ur7e_perception_planning/config/demo.yaml').read_text())
    config['/ur7e_nvblox']['ros__parameters']['esdf_hz']=args.esdf_hz
    (out/'config.yaml').write_text(yaml.safe_dump(config))
    launch=observer=node=None;results={}
    try:
        with (out/'launch.log').open('w') as log:
            launch=subprocess.Popen(['/opt/ros/humble/bin/ros2','launch','ur7e_perception_planning','bringup.launch.py',
                'run_rviz:=false','config:='+str(out/'config.yaml')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init();node=rclpy.create_node('learning_benchmark_goal');pub=node.create_publisher(PoseStamped,'/target_pose',1)
        for name,goal in [('baseline',[.5,0.,.45]),('wall',[.45,.4,.55])]:
            with (out/(name+'.log')).open('w') as log:
                observer=subprocess.Popen([sys.executable,'-B',str(Path(__file__).with_name('validate_demo.py')),
                    '--output',str(out/(name+'.json')),'--duration',str(args.duration),'--goal',*map(str,goal)],
                    env=env,stdout=log,stderr=subprocess.STDOUT)
            if name=='wall':
                deadline=time.monotonic()+2
                while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
                msg=PoseStamped();msg.header.frame_id='world';msg.pose.orientation.w=1.
                msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=goal;pub.publish(msg)
            observer.wait(timeout=args.duration+20)
            data=json.loads((out/(name+'.json')).read_text())
            results[name]={k:data[k] for k in ['passed','duration','latest','min_geometry_clearance','map_rate']}
            if name=='wall':
                results[name]['detour']=wall_detour_check(data,goal)
                results[name]['passed'] &= results[name]['detour']['passed']
            (out/'summary.json').write_text(json.dumps(results,indent=2))
            print(name,json.dumps(results[name]),flush=True)
            if not results[name]['passed']:return 1
        return 0
    finally:
        if observer and observer.poll() is None:observer.terminate();observer.wait(timeout=5)
        if launch and launch.poll() is None:
            launch.send_signal(signal.SIGINT)
            try:launch.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=10)
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
