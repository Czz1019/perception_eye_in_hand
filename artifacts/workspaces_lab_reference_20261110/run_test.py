import os,time,subprocess,signal,json,shutil
from pathlib import Path
root=Path('/home/czz/perception_eye_in_hand');out=Path(__file__).resolve().parent;ws=root/'ur7e_nvblox_in_hand';children=[]
def spawn(command,envfile,log,extra=None):
 f=open(out/log,'w');p=subprocess.Popen(['bash','-c','source '+str(envfile)+'\nexport ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1 GAZEBO_MASTER_URI=http://127.0.0.1:11566\n'+command],stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env={**os.environ,**(extra or {})});f.close();children.append(p);return p
def stop(p):
 if p.poll() is not None:return
 os.killpg(p.pid,signal.SIGINT)
 try:p.wait(12)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(5)
def save(folder):
 folder.mkdir(exist_ok=True)
 command='\n'.join(f"timeout 35 ros2 service call /nvblox_node/{s} nvblox_msgs/srv/FilePath '{{file_path: \"{folder/f}\"}}'" for s,f in [('save_ply','mesh.ply'),('save_map','map.blox'),('save_rates','rates.txt'),('save_timings','timings.txt')])+'\nros2 param dump /nvblox_node > '+str(folder/'params.yaml')
 p=spawn(command,root/'tools/workspaces_lab_reference/env.sh',folder.name+'_save.log');p.wait(140)
 if not (folder/'mesh.ply').exists():raise RuntimeError('Mesh export failed '+folder.name)
 p=spawn('python3 '+str(out/'query_esdf.py')+' '+str(folder),root/'tools/workspaces_lab_reference/env.sh',folder.name+'_esdf.log');p.wait(45)
 if p.returncode:raise RuntimeError('ESDF query failed')
 for f in ['capture.json','depth_m.npy','T_base_depth.npy','rgb.png']:shutil.copy2(out/f,folder/f)
try:
 scene=spawn('ros2 launch ur7e_d435_gazebo lab_scene.launch.py gazebo_gui:=false',ws/'env.sh','scene.log');time.sleep(13)
 mapper=spawn('ros2 launch nvblox_examples_bringup ur7e_lab_tf.launch.py',root/'tools/workspaces_lab_reference/env.sh','official_mapper.log');time.sleep(12)
 if mapper.poll() is not None:raise RuntimeError('Official launch exited')
 cap=spawn('python3 '+str(ws/'artifacts/lab_accuracy_20261110/capture.py'),ws/'env.sh','capture.log',{'UR7E_LAB_TEST_OUT':str(out)});cap.wait(30)
 if cap.returncode:raise RuntimeError('Capture failed')
 print('Static observation 25 seconds',flush=True);time.sleep(25);save(out/'static');print('Static exported',flush=True)
 motion=spawn('python3 '+str(ws/'artifacts/lab_multiview_20261110/execute_motion.py'),ws/'env.sh','motion.log',{'UR7E_LAB_TEST_OUT':str(out)});motion.wait(240)
 if motion.returncode:raise RuntimeError('Motion failed')
 time.sleep(5);save(out/'multiview');print('Multiview exported',flush=True)
finally:
 for p in reversed(children):stop(p)
