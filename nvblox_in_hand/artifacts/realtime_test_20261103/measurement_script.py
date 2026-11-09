import json, os, signal, subprocess, time, statistics, traceback
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import Marker

root=Path('/home/czz/perception_eye_in_hand/nvblox_in_hand')
out=root/'artifacts/realtime_test_20261103'
out.mkdir(parents=True,exist_ok=True)
result={'test':'stationary_gazebo_simple','arm_motion_commands_sent':0,
        'gpu_completion_measured':False,'phases':{}}
log=(out/'launch.log').open('w')
proc=None
node=None
samples=[]
clocks=[]
markers=[]

def metric(msg):
    try: samples.append((time.monotonic(),json.loads(msg.data)))
    except Exception: pass
def clock(msg): clocks.append((time.monotonic(),msg.clock.sec+msg.clock.nanosec*1e-9))
def marker(msg): markers.append((time.monotonic(),len(msg.points)))
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if proc.poll() is not None: raise RuntimeError('launch exited: '+str(proc.returncode))
        rclpy.spin_once(node,timeout_sec=.1)
def summary(start,end):
    a=[(t,m) for t,m in samples if start<=t<=end]
    c=[(t,s) for t,s in clocks if start<=t<=end]
    v=[(t,n) for t,n in markers if start<=t<=end]
    d={'wall_seconds':end-start,'metrics_samples':len(a),'marker_messages':len(v)}
    if len(a)>1:
        dt=a[-1][0]-a[0][0]
        for name,key in [('depth','depth_integrated_total'),('color','color_integrated_total'),('esdf_cycles','esdf_cycles_total')]:
            d[name+'_wall_hz']=(a[-1][1][key]-a[0][1][key])/dt
        for key in ['esdf_process_host_ms','esdf_view_host_ms']:
            x=sorted(m[key] for _,m in a)
            d[key+'_sample_mean']=statistics.mean(x)
            d[key+'_sample_min']=min(x)
            d[key+'_sample_max']=max(x)
        d['last_metrics']=a[-1][1]
    if len(c)>1:
        d['gazebo_sim_time_per_wall_second']=(c[-1][1]-c[0][1])/(c[-1][0]-c[0][0])
    if v:
        d['marker_points_min']=min(n for _,n in v)
        d['marker_points_max']=max(n for _,n in v)
    if len(v)>1: d['marker_wall_hz']=(len(v)-1)/(v[-1][0]-v[0][0])
    return d

try:
    rclpy.init()
    node=Node('in_hand_simple_realtime_measurement')
    node.create_subscription(String,'/nvblox_node/metrics',metric,10)
    node.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data)
    cmd=[str(root/'run.sh'),'gazebo_gui:=false','run_rviz:=false',
         'mapper_backend:=observed_view','esdf_mode:=3d','enable_esdf:=true',
         'enable_esdf_view:=true','voxel_size:=0.02','max_range_m:=3.0']
    result['command']=cmd
    proc=subprocess.Popen(cmd,cwd=root,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    print('Started scene and mapper; waiting for integrated depth.',flush=True)
    until=time.monotonic()+75
    while not samples or samples[-1][1].get('depth_integrated_total',0)<10:
        if time.monotonic()>until: raise RuntimeError('No integrated depth within 75 seconds')
        spin(.2)
    spin(5)
    dump=subprocess.run(['ros2','param','dump','/nvblox_node'],capture_output=True,text=True,timeout=12)
    (out/'runtime_parameters.yaml').write_text(dump.stdout+dump.stderr)
    for name,duration in [('no_esdf_display_subscriber',25),('esdf_marker_subscribed',20)]:
        if name=='esdf_marker_subscribed':
            view=node.create_subscription(Marker,'/nvblox_node/esdf_native_band',marker,2)
            spin(2)
        print('Measuring '+name+' for '+str(duration)+' seconds.',flush=True)
        began=time.monotonic();spin(duration);ended=time.monotonic()
        result['phases'][name]=summary(began,ended)
        print(json.dumps(result['phases'][name],ensure_ascii=False),flush=True)
    result['status']='completed'
except Exception as e:
    result['status']='failed';result['error']=str(e)
    traceback.print_exc()
finally:
    (out/'samples.json').write_text(json.dumps({'metrics':samples,'clock':clocks,'markers':markers},indent=2))
    (out/'results.json').write_text(json.dumps(result,indent=2,ensure_ascii=False))
    if proc and proc.poll() is None:
        os.killpg(proc.pid,signal.SIGINT)
        try: proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGTERM)
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    log.close()
    if node:node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    print('Results: '+str(out/'results.json'),flush=True)
