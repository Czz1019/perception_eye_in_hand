import json, os, signal, subprocess, time, statistics, traceback, copy
from pathlib import Path
import numpy as np
import yaml
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import Marker
from nvblox_msgs.msg import Mesh
from cdr_marker_count import marker_points_count
from nvblox_msgs.srv import FilePath
from tf2_ros import Buffer, TransformListener

root=Path('/home/czz/perception_eye_in_hand/nvblox_in_hand')
out=root/'artifacts/rate30_test_20261104'
result={'status':'running','probe_mode':'raw_cdr_marker_and_raw_mesh','gpu_completion_measured':False,'phases':{},'commands':[],
        'motion':'previous_observation_pose_and_wrist1_sweep_0.06rad','screenshots':[],'requested_rates_hz':{'depth':30,'color':30,'esdf':30,'mesh':30,'layer':30,'esdf_view':30}}
processes=[];logs=[];critical=[];samples=[];clocks=[];mesh_times=[];marker_times=[];rgb_times=[];depth_times=[];camera_poses=[]
rgb=None;depth=None;camera_info=None
rclpy.init();node=Node('quality_realtime_measurement')
tf=Buffer();listener=TransformListener(tf,node)
def metric(m):
    try:samples.append((time.monotonic(),json.loads(m.data)))
    except Exception:pass
def rgb_cb(m):
    global rgb
    rgb=m
    rgb_times.append((time.monotonic(),m.header.stamp.sec+m.header.stamp.nanosec*1e-9))
def depth_cb(m):
    global depth
    depth=m
    depth_times.append((time.monotonic(),m.header.stamp.sec+m.header.stamp.nanosec*1e-9))
def info_cb(m):
    global camera_info
    camera_info=m
node.create_subscription(String,'/nvblox_node/metrics',metric,10)
node.create_subscription(Clock,'/clock',lambda m:clocks.append((time.monotonic(),m.clock.sec+m.clock.nanosec*1e-9)),qos_profile_sensor_data)
node.create_subscription(Mesh,'/nvblox_node/mesh',lambda m:mesh_times.append((time.monotonic(),len(m),0)),2,raw=True)
node.create_subscription(Image,'/camera/color/image_raw',rgb_cb,qos_profile_sensor_data)
node.create_subscription(Image,'/camera/depth/image_rect_raw',depth_cb,qos_profile_sensor_data)
node.create_subscription(CameraInfo,'/camera/depth/camera_info',info_cb,qos_profile_sensor_data)
def start(cmd,name,required=False):
    f=(out/(name+'.log')).open('w');logs.append(f)
    p=subprocess.Popen(cmd,cwd=root,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(p)
    if required:critical.append(p)
    result['commands'].append({'name':name,'argv':cmd,'pid':p.pid})
    print('Started '+name,flush=True);return p
def stop(p):
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        for p in critical:
            if p.poll() is not None:raise RuntimeError('Required process exited '+str(p.pid))
        rclpy.spin_once(node,timeout_sec=.05)
def wait(p,timeout=70):
    end=time.monotonic()+timeout
    while p.poll() is None and time.monotonic()<end:spin(.1)
    if p.poll()!=0:raise RuntimeError('Command failed/timed out '+str(p.pid)+' exit '+str(p.poll()))
def screenshot(name):
    fn=out/(name+'.png')
    screenshot_env=os.environ.copy()
    for key in ['LD_LIBRARY_PATH','PYTHONPATH','QT_PLUGIN_PATH','QT_QPA_PLATFORM_PLUGIN_PATH']:
        screenshot_env.pop(key,None)
    p=subprocess.run(['/usr/bin/python3','-s',str(out/'capture_window.py'),str(fn)],env=screenshot_env,capture_output=True,text=True,timeout=10)
    (out/(name+'_capture.txt')).write_text(p.stdout+p.stderr)
    if p.returncode:raise RuntimeError('Capture failed '+p.stderr)
    result['screenshots'].append(str(fn));print('Captured '+name,flush=True)
def raw(name):
    if rgb:
        a=np.frombuffer(rgb.data,dtype=np.uint8).reshape(rgb.height,rgb.step)[:,:rgb.width*3].reshape(rgb.height,rgb.width,3)
        if rgb.encoding=='bgr8':a=a[:,:,::-1]
        PILImage.fromarray(a).save(out/(name+'_rgb.png'))
    if depth:
        dtype=np.uint16 if depth.encoding=='16UC1' else np.float32
        a=np.frombuffer(depth.data,dtype=dtype).reshape(depth.height,depth.step//np.dtype(dtype).itemsize)[:,:depth.width]
        np.save(out/(name+'_depth.npy'),a)
        result.setdefault('depth_inputs',{})[name]={'width':depth.width,'height':depth.height,'encoding':depth.encoding,'frame':depth.header.frame_id,'positive_fraction':float(np.mean(a>0))}
    if camera_info:
        result['camera_info']={'k':list(camera_info.k),'width':camera_info.width,'height':camera_info.height,'frame':camera_info.header.frame_id}
def save_mesh(name):
    c=node.create_client(FilePath,'/nvblox_node/save_ply')
    if not c.wait_for_service(timeout_sec=3):raise RuntimeError('save_ply unavailable')
    req=FilePath.Request();req.file_path=str(out/(name+'.ply'));f=c.call_async(req)
    end=time.monotonic()+25
    while not f.done() and time.monotonic()<end:spin(.1)
    if not f.done() or not f.result().success:raise RuntimeError('Mesh save failed')
    node.destroy_client(c)
def summarize(name,began,ended):
    a=[(t,m) for t,m in samples if began<=t<=ended]
    c=[(t,s) for t,s in clocks if began<=t<=ended]
    d={'start_monotonic':began,'end_monotonic':ended,'wall_seconds':ended-began,'sample_count':len(a)}
    if len(a)>1:
        dt=a[-1][0]-a[0][0]
        for n,k in [('depth','depth_integrated_total'),('color','color_integrated_total'),('esdf_cycles','esdf_cycles_total')]:d[n+'_wall_hz']=(a[-1][1][k]-a[0][1][k])/dt
        for k in ['esdf_process_host_ms','esdf_view_host_ms']:
            x=[m[k] for _,m in a];d[k+'_sample_mean']=statistics.mean(x);d[k+'_sample_p95']=float(np.percentile(x,95))
        d['last_metrics']=a[-1][1]
    for key,vals in [('mesh',mesh_times),('esdf_marker',marker_times),('rgb_input',rgb_times),('depth_input',depth_times)]:
        ts=[x[0] for x in vals if began<=x[0]<=ended]
        d[key+'_messages']=len(ts)
        if len(ts)>1:d[key+'_wall_hz']=(len(ts)-1)/(ts[-1]-ts[0])
    points=[n for t,n in marker_times if began<=t<=ended]
    if points:d['esdf_marker_points']={'min':min(points),'max':max(points),'mean':statistics.mean(points)}
    if len(c)>1:d['simulation_rtf']=(c[-1][1]-c[0][1])/(c[-1][0]-c[0][0])
    result['phases'][name]=d;print(name+': '+json.dumps(d),flush=True)
def phase(name,duration,motion=False):
    print('Measuring '+name,flush=True);p=None
    if motion:p=start([str(root.parent/'run_ur7e_view_sweep.sh'),'--seconds',str(duration),'--amplitude','0.06'],name+'_motion')
    began=time.monotonic();spin(duration*.55);screenshot(name);raw(name);spin(duration*.45)
    if p:wait(p,25)
    summarize(name,began,time.monotonic())

cfg=yaml.safe_load((root/'src/nvblox_in_hand/rviz/in_hand.rviz').read_text())
for disp in cfg['Visualization Manager']['Displays']:
    if disp['Name']=='D435 RGB input':disp['Enabled']=True;disp['Value']=True
cfg['Window Geometry']={'Width':1440,'Height':900,'X':0,'Y':0}
for esdf,name in [(False,'mesh_view'),(True,'esdf_view')]:
    v=copy.deepcopy(cfg)
    for d in v['Visualization Manager']['Displays']:
        if d['Name'].startswith('Observed native ESDF'):d['Enabled']=esdf;d['Value']=esdf
    (out/(name+'.rviz')).write_text(yaml.safe_dump(v,sort_keys=False))

try:
    scene=start(['ros2','launch','ur7e_d435_gazebo','scene.launch.py','gazebo_gui:=false'],'scene',True)
    check=start([str(root.parent/'run_ur7e_view_sweep.sh'),'--check-only'],'scene_check');wait(check,45)
    approach=start([str(root.parent/'run_ur7e_view_sweep.sh'),'--seconds','0'],'approach');wait(approach,70)
    mapper=start([str(root/'run.sh'),'start_scene:=false','run_rviz:=false','mapper_backend:=observed_view','esdf_mode:=3d','enable_esdf_view:=true','voxel_size:=0.02','max_range_m:=3.0','depth_hz:=30.0','color_hz:=30.0','esdf_hz:=30.0','mesh_hz:=30.0','layer_hz:=30.0','esdf_view_hz:=30.0'],'mapping',True)
    end=time.monotonic()+60
    while not samples or samples[-1][1].get('depth_integrated_total',0)<30:
        if time.monotonic()>end:raise RuntimeError('Mapper no depth integration')
        spin(.1)
    rviz=start(['rviz2','-d',str(out/'mesh_view.rviz'),'--ros-args','-p','use_sim_time:=true'],'rviz_mesh');spin(8)
    dump=subprocess.run(['ros2','param','dump','/nvblox_node'],capture_output=True,text=True,timeout=10)
    (out/'runtime_parameters.yaml').write_text(dump.stdout+dump.stderr)
    try:
        t=tf.lookup_transform('world','base_link',Time())
        result['world_from_base_link']={'translation':[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],
            'quaternion_xyzw':[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]}
    except Exception as e:result['world_transform_error']=str(e)
    phase('mesh_static',20);save_mesh('mesh_static')
    phase('mesh_wrist_motion',20,True)
    phase('mesh_stopped',12);save_mesh('mesh_after_motion')
    stop(rviz)
    node.create_subscription(Marker,'/nvblox_node/esdf_native_band',lambda m:marker_times.append((time.monotonic(),marker_points_count(m))),2,raw=True)
    rviz=start(['rviz2','-d',str(out/'esdf_view.rviz'),'--ros-args','-p','use_sim_time:=true'],'rviz_esdf');spin(6)
    phase('mesh_esdf_static',20)
    phase('mesh_esdf_wrist_motion',20,True)
    phase('mesh_esdf_stopped',12);save_mesh('mesh_final')
    result['status']='completed'
except Exception as e:
    result['status']='failed';result['error']=str(e);traceback.print_exc()
finally:
    (out/'results.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    (out/'samples.json').write_text(json.dumps({'metrics':samples,'clock':clocks,'mesh':mesh_times,'esdf_markers':marker_times,'rgb_input':rgb_times,'depth_input':depth_times}))
    for p in reversed(processes):
        try:stop(p)
        except Exception:traceback.print_exc()
    for f in logs:f.close()
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    print('Finished: '+str(out/'results.json'),flush=True)
