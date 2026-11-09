"""Real Gazebo validation of the refactored example chain, domain 129 only."""
import os,signal,subprocess,time,json,traceback,statistics,re
from pathlib import Path
import numpy as np
import yaml
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from nvblox_msgs.msg import Mesh
from nvblox_msgs.srv import FilePath,EsdfAndGradients
from tf2_ros import Buffer,TransformListener

root=Path('/home/czz/perception_eye_in_hand/nvblox_in_hand');out=Path(__file__).resolve().parent
assert os.environ.get('ROS_DOMAIN_ID')=='129'
processes=[];logs=[];critical=[]
result={'status':'running','phases':{},'commands':[],'geometry':{'voxel_size_m':.03,'max_range_m':3.,'input_resolution':[640,480]},'sensor_target_hz':{'depth':30,'color':30},'configured_rates_hz':{'fusion_limit':40,'baseline_esdf':30,'trial_esdf':33.4,'mesh_layer':5,'baseline_view':30,'trial_views':[33.4], 'palette_only_comparison':True},'motion':'above-obstacle observation pose, wrist_1 +/-0.08 rad, 0.4 rad/s sine phase, 30 wall seconds; no orbit','gpu_completion_measured':False}
samples=[];clocks=[];mesh_times=[];cloud_times=[];slice_times=[];rgb_times=[];depth_times=[]
rgb=depth=info=last_cloud=last_slice=None
camera_trajectory=[]
rclpy.init();node=Node('refactored_example_measurement');buffer=Buffer();listener=TransformListener(buffer,node)
def stamp(h):return h.stamp.sec+h.stamp.nanosec*1e-9
def metric(m):
 try:samples.append((time.monotonic(),json.loads(m.data)))
 except (ValueError,TypeError):pass
def rgb_cb(m):
 global rgb
 rgb=m;rgb_times.append((time.monotonic(),stamp(m.header)))
def depth_cb(m):
 global depth
 depth=m;depth_times.append((time.monotonic(),stamp(m.header)))
def info_cb(m):
 global info
 info=m
def cloud_cb(m):
 global last_cloud
 last_cloud=m;cloud_times.append((time.monotonic(),m.width*m.height,clocks[-1][1]-stamp(m.header) if clocks else None))
def slice_cb(m):
 global last_slice
 last_slice=m;slice_times.append((time.monotonic(),m.width*m.height))
node.create_subscription(Clock,'/clock',lambda m:clocks.append((time.monotonic(),m.clock.sec+m.clock.nanosec*1e-9)),qos_profile_sensor_data)
node.create_subscription(Image,'/camera/color/image_raw',rgb_cb,qos_profile_sensor_data)
node.create_subscription(Image,'/camera/depth/image_rect_raw',depth_cb,qos_profile_sensor_data)
node.create_subscription(CameraInfo,'/camera/depth/camera_info',info_cb,qos_profile_sensor_data)
color_info=None
def color_info_cb(m):
 global color_info
 color_info=m
node.create_subscription(CameraInfo,'/camera/color/camera_info',color_info_cb,qos_profile_sensor_data)
node.create_subscription(Mesh,'/nvblox_node/mesh',lambda m:mesh_times.append((time.monotonic(),len(m))),2,raw=True)
node.create_subscription(PointCloud2,'/nvblox_node/static_esdf_pointcloud',slice_cb,2)
node.create_subscription(PointCloud2,'/nvblox_node/esdf_points',cloud_cb,2)
node.create_subscription(String,'/nvblox_node/metrics',metric,10)
def start(cmd,name,required=False):
 f=(out/(name+'.log')).open('w');logs.append(f)
 p=subprocess.Popen(cmd,cwd=root,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
 processes.append(p)
 if required:critical.append(p)
 result['commands'].append({'name':name,'argv':cmd,'pid':p.pid});print('START '+name,flush=True);return p
def stop(p):
 if p.poll() is None:
  p.send_signal(signal.SIGINT)
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=4)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  for p in critical:
   if p.poll() is not None:raise RuntimeError('Required process exited '+str(p.pid))
  rclpy.spin_once(node,timeout_sec=.02)
  if not camera_trajectory or time.monotonic()-camera_trajectory[-1]['wall_time']>.1:
   try:
    tr=buffer.lookup_transform('world','camera_depth_optical_frame',Time())
    q=tr.transform.rotation;v=tr.transform.translation
    from scipy.spatial.transform import Rotation
    camera_trajectory.append({'wall_time':time.monotonic(),'stamp':stamp(tr.header),'position':[v.x,v.y,v.z],'optical_forward':Rotation.from_quat([q.x,q.y,q.z,q.w]).apply([0.,0.,1.]).tolist()})
   except Exception:pass
def wait(p,timeout):
 end=time.monotonic()+timeout
 while p.poll() is None and time.monotonic()<end:spin(.05)
 if p.poll()!=0:raise RuntimeError('Command failed or timeout, pid='+str(p.pid)+' code='+str(p.poll()))
def file_service(service,path,timeout=25):
 c=node.create_client(FilePath,'/nvblox_node/'+service)
 if not c.wait_for_service(timeout_sec=3):raise RuntimeError(service+' unavailable')
 req=FilePath.Request();req.file_path=str(path);future=c.call_async(req);end=time.monotonic()+timeout
 while not future.done() and time.monotonic()<end:spin(.05)
 if not future.done() or not future.result() or not future.result().success:raise RuntimeError(service+' failed')
 node.destroy_client(c)
def parse_rates(path):
 text=path.read_text();rates={}
 for name,count,hz in re.findall(r'^\s*(ros/\S+)\s+(\d+)\s+([-\d.]+)',text,re.M):rates[name]={'window_samples':int(count),'hz':float(hz)}
 return rates

def screenshot(name):
 env=os.environ.copy()
 for key in ('LD_LIBRARY_PATH','PYTHONPATH','QT_PLUGIN_PATH','QT_QPA_PLATFORM_PLUGIN_PATH'):env.pop(key,None)
 p=subprocess.run(['/usr/bin/python3','-s',str(out/'capture_rviz_composite.py'),str(out/(name+'.png'))],env=env,capture_output=True,text=True,timeout=10)
 (out/(name+'_capture.txt')).write_text(p.stdout+p.stderr)
 if p.returncode:raise RuntimeError('Owned RViz screenshot failed '+p.stderr)
 if rgb:
  a=np.frombuffer(rgb.data,dtype=np.uint8).reshape(rgb.height,rgb.step)[:,:rgb.width*3].reshape(rgb.height,rgb.width,3)
  if rgb.encoding=='bgr8':a=a[:,:,::-1]
  PILImage.fromarray(a).save(out/(name+'_rgb.png'))
 if depth:
  typ=np.uint16 if depth.encoding=='16UC1' else np.float32
  a=np.frombuffer(depth.data,dtype=typ).reshape(depth.height,depth.step//np.dtype(typ).itemsize)[:,:depth.width]
  np.save(out/(name+'_depth.npy'),a)
 print('CAPTURE '+name,flush=True)
def summarize(name,began,ended):
 d={'wall_seconds':ended-began,'start_monotonic':began,'end_monotonic':ended}
 for key,events in [('mesh',mesh_times),('esdf_cloud',cloud_times),('esdf_slice',slice_times),('rgb_input',rgb_times),('depth_input',depth_times)]:
  e=[x for x in events if began<=x[0]<=ended];d[key+'_messages']=len(e)
  if len(e)>1:d[key+'_wall_hz']=(len(e)-1)/(e[-1][0]-e[0][0])
  if e and key in ('esdf_cloud','esdf_slice'):d[key+'_points']={'min':min(x[1] for x in e),'max':max(x[1] for x in e),'mean':statistics.mean(x[1] for x in e)}
 c=[x for x in clocks if began<=x[0]<=ended]
 if len(c)>1:d['simulation_rtf']=(c[-1][1]-c[0][1])/(c[-1][0]-c[0][0])
 s=[x for x in samples if began<=x[0]<=ended]
 if len(s)>1:
  dt=s[-1][0]-s[0][0]
  for k,label in [('depth_integrated','depth'),('color_integrated','color'),('esdf_updated','esdf_update')]:d['diagnostic_'+label+'_wall_hz']=(s[-1][1]['totals'][k]-s[0][1]['totals'][k])/dt
  for key in ('esdf_process_host_ms','esdf_view_host_ms'):
   values=[x[1][key] for x in s];d[key+'_sample_mean']=float(np.mean(values));d[key+'_sample_p95']=float(np.percentile(values,95))
 a=[x[2] for x in cloud_times if began<=x[0]<=ended and x[2] is not None]
 if a:d['cloud_age_latest_clock_sec']={'p50':float(np.percentile(a,50)),'p95':float(np.percentile(a,95))}
 result['phases'][name]=d;print('PHASE '+name+' '+json.dumps(d),flush=True)
 rates=out/(name+'_rates.txt');file_service('save_rates',rates)
 d['official_rolling_rates']=parse_rates(rates)
 file_service('save_timings',out/(name+'_timings.txt'))
def phase(name,seconds,motion=False):
 began=time.monotonic();p=None
 if motion:p=start(['/usr/bin/python3',str(out/'view_sweep_isolated.py'),'--seconds',str(seconds)],name+'_motion')
 spin(seconds*.6);screenshot(name);spin(seconds*.4)
 if p:wait(p,25)
 summarize(name,began,time.monotonic())
def cloud_snapshot(name,m):
 if m is None:raise RuntimeError('No cloud for '+name)
 fields={f.name:(f.offset,f.datatype) for f in m.fields}
 dtype=np.dtype({'names':['x','y','z','distance','observed','inside','site'], 'formats':['<f4']*4+['u1']*3,'offsets':[0,4,8,12,20,21,22],'itemsize':24})
 assert m.point_step==24 and fields['observed']==(20,2) and fields['is_inside']==(21,2) and fields['is_site']==(22,2)
 assert len(m.data)==m.row_step*m.height
 a=np.frombuffer(m.data,dtype=dtype);xyz=np.stack([a['x'],a['y'],a['z']],axis=1)
 assert np.isfinite(xyz).all() and np.isfinite(a['distance']).all() and np.all(a['observed']==1)
 assert np.max(np.abs(xyz/.03-.5-np.round(xyz/.03-.5)),initial=0)<1e-3
 assert np.all(a['distance'][a['inside']==1]<=0) and np.all(a['distance'][a['inside']==0]>=0)
 np.save(out/(name+'_esdf.npy'),a)
 return {'points':len(a),'inside':int(np.sum(a['inside']==1)),'outside':int(np.sum(a['inside']==0)),'sites':int(np.sum(a['site']==1)),'point_step':m.point_step,'frame':m.header.frame_id,'distance_min':float(a['distance'].min()),'distance_max':float(a['distance'].max()),'validation':'24-byte layout, explicit flags, finite native centres and signed distances passed'}
def query_esdf(name):
 c=node.create_client(EsdfAndGradients,'/nvblox_node/get_esdf_and_gradient')
 if not c.wait_for_service(timeout_sec=3):raise RuntimeError('Native ESDF service unavailable')
 req=EsdfAndGradients.Request();req.update_esdf=False;req.visualize_esdf=False;req.use_aabb=True;req.frame_id='base_link'
 req.aabb_min_m.x=-1.;req.aabb_min_m.y=-1.;req.aabb_min_m.z=-.2
 req.aabb_size_m.x=2.;req.aabb_size_m.y=2.;req.aabb_size_m.z=1.4
 began=time.monotonic();f=c.call_async(req);end=began+25
 while not f.done() and time.monotonic()<end:spin(.05)
 if not f.done() or not f.result() or not f.result().success:raise RuntimeError('Native ESDF query failed')
 response=f.result();roundtrip_ms=(time.monotonic()-began)*1000;a=np.asarray(response.esdf_and_gradients.data,dtype=np.float32);np.save(out/(name+'_native_esdf.npy'),a)
 known=np.isfinite(a)&(a!=-1000.)
 d={'host_roundtrip_ms':roundtrip_ms,'voxel_size_m':response.voxel_size_m,'origin_m':[response.origin_m.x,response.origin_m.y,response.origin_m.z], 'shape':[x.size for x in response.esdf_and_gradients.layout.dim],'values':len(a),'known':int(known.sum()),'unknown':int((~known).sum()),'known_positive':int(np.sum(known&(a>0))),'known_negative':int(np.sum(known&(a<0))),'known_zero':int(np.sum(known&(a==0))),'query_outside_rate_measurement':True}
 node.destroy_client(c);return d
try:
 scene=start(['ros2','launch','ur7e_d435_gazebo','scene.launch.py','gazebo_gui:=false'],'scene',True)
 wait(start(['/usr/bin/python3',str(out/'view_sweep_isolated.py'),'--check-only'],'scene_ready'),45)
 wait(start(['/usr/bin/python3',str(out/'view_sweep_isolated.py'),'--seconds','0'],'approach'),70)
 spin(1)
 if not camera_trajectory:raise RuntimeError('No verified camera pose')
 result['camera_observation_pose']=camera_trajectory[-1]
 if camera_trajectory[-1]['position'][2]<=.5 or camera_trajectory[-1]['optical_forward'][2]>=0:raise RuntimeError('Observation camera is not above wall top and downward-facing: '+str(camera_trajectory[-1]))
 audit=start([str(root/'check_sim.sh'),'--output',str(out/'robot_state_audit.json')],'robot_state_audit');wait(audit,35)
 for mode in ('overhead',):
  last_cloud=last_slice=None
  backend='diagnostic';esdf_mode='3d'
  before_mesh=len(mesh_times)
  esdf_hz=33.4
  view_hz=33.4
  extra=['esdf_hz:='+str(esdf_hz),'esdf_view_hz:='+str(view_hz)]
  mapper=start([str(root/'run.sh'),'start_scene:=false','run_rviz:=false','mapper_backend:='+backend,'esdf_mode:='+esdf_mode,'profile:=realtime','esdf_view_mode:=observed']+extra,'mapping_'+mode,True)
  end=time.monotonic()+65
  while len(mesh_times)<before_mesh+10:
   if time.monotonic()>end:raise RuntimeError('No native mesh after startup '+mode)
   spin(.1)
  rviz=start(['rviz2','-d',str(out/(mode+'.rviz')),'--ros-args','-p','use_sim_time:=true'],'rviz_'+mode,True);spin(5)
  dump=subprocess.run(['ros2','param','dump','/nvblox_node'],capture_output=True,text=True,timeout=10)
  if dump.returncode:raise RuntimeError('Parameter dump failed')
  (out/(mode+'_runtime_parameters.yaml')).write_text(dump.stdout)
  actual_params=yaml.safe_load(dump.stdout)['/nvblox_node']['ros__parameters']
  expected=40.
  assert actual_params['integrate_depth_rate_hz']==expected and actual_params['integrate_color_rate_hz']==expected
  assert actual_params['esdf_mode']=='3d' and abs(actual_params['voxel_size']-.03)<1e-6
  assert abs(actual_params['update_esdf_rate_hz']-esdf_hz)<1e-4 and actual_params['publish_layer_rate_hz']==5.
  if backend=='diagnostic':assert abs(actual_params['esdf_view_hz']-view_hz)<1e-4

  comp=subprocess.run(['ros2','component','list'],capture_output=True,text=True,timeout=10)
  (out/(mode+'_components.txt')).write_text(comp.stdout+comp.stderr)
  result.setdefault('backends',{})[mode]={'custom_cloud_publishers':node.count_publishers('/nvblox_node/esdf_points'),'custom_metrics_publishers':node.count_publishers('/nvblox_node/metrics'),'mesh_publishers':node.count_publishers('/nvblox_node/mesh'),'component_list':comp.stdout}
  if backend=='official':assert result['backends'][mode]['custom_cloud_publishers']==0 and result['backends'][mode]['custom_metrics_publishers']==0
  assert result['backends'][mode]['custom_cloud_publishers']==1
  assert depth is not None and rgb is not None and info is not None and color_info is not None
  assert (depth.width,depth.height)==(640,480) and (rgb.width,rgb.height)==(640,480)
  assert (info.width,info.height)==(640,480) and (color_info.width,color_info.height)==(640,480)
  inputs={}
  for label,image,calib in [('depth',depth,info),('color',rgb,color_info)]:
   assert image.header.frame_id==calib.header.frame_id and calib.k[0]>0 and calib.k[4]>0
   assert abs(calib.k[2]-320)<1e-3 and abs(calib.k[5]-240)<1e-3
   inputs[label]={'width':image.width,'height':image.height,'encoding':image.encoding,'frame':image.header.frame_id,'camera_info_width':calib.width,'camera_info_height':calib.height,'k':list(calib.k),'p':list(calib.p),'vertical_fov_rad':2*np.arctan(calib.height/(2*calib.k[4]))}
  result['backends'][mode]['actual_inputs']=inputs
  phase(mode+'_static',10)
  file_service('save_ply',out/'mesh_before.ply')
  result['before_scan_cloud']=cloud_snapshot('before',last_cloud)
  phase(mode+'_wrist_motion',30,True)
  phase(mode+'_stopped',8)
  file_service('save_ply',out/('mesh_'+mode+'.ply'))
  file_service('save_map',out/('map_'+mode+'.nvblx'))
  if esdf_mode=='3d':result['backends'][mode]['native_esdf_query']=query_esdf(mode)
  if backend=='diagnostic':result['backends'][mode]['cloud_snapshot']=cloud_snapshot(mode,last_cloud)
  if mode=='official2d':
   assert last_slice and last_slice.width>0
   result['backends'][mode]['slice_snapshot']={'points':last_slice.width*last_slice.height,'point_step':last_slice.point_step,'frame':last_slice.header.frame_id}
  tf=buffer.lookup_transform('world','base_link',Time())
  result['world_from_base_link']={'translation':[tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z],'quaternion_xyzw':[tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w]}
  critical.remove(rviz);stop(rviz)
  rviz=start(['rviz2','-d',str(out/'mesh.rviz'),'--ros-args','-p','use_sim_time:=true'],'rviz_mesh',True);spin(5);screenshot('mesh_after')
  critical.remove(rviz);stop(rviz);critical.remove(mapper);stop(mapper);spin(1)
 result['status']='completed'
except Exception as error:
 result['status']='failed';result['error']=str(error);traceback.print_exc()
finally:
 (out/'camera_trajectory.json').write_text(json.dumps(camera_trajectory,indent=2)+'\n')
 (out/'results.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 (out/'samples.json').write_text(json.dumps({'metrics':samples,'clocks':clocks,'mesh':mesh_times,'cloud':cloud_times,'slice':slice_times,'rgb':rgb_times,'depth':depth_times}))
 for p in reversed(processes):
  try:stop(p)
  except Exception:traceback.print_exc()
 for f in logs:f.close()
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 print('DONE '+result['status'],flush=True)
