#!/usr/bin/env python3
"""Isolated synthetic Mesh rendering check; never starts Gazebo or sends arm commands."""
import json
import os
from pathlib import Path
import signal
import resource
import subprocess
import time
import yaml
import rclpy
from geometry_msgs.msg import Point32,TransformStamped
from nvblox_msgs.msg import Mesh,MeshBlock,Index3D
from std_msgs.msg import ColorRGBA
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

root=Path(__file__).resolve().parents[1]
out=root/'artifacts/rviz_fix_20261028';out.mkdir(exist_ok=True)
config=yaml.safe_load((root/'src/nvblox_in_hand/rviz/in_hand.rviz').read_text())
manager=config['Visualization Manager'];manager['Global Options']['Fixed Frame']='base_link'
manager['Displays']=[d for d in manager['Displays'] if d.get('Class')=='nvblox_rviz_plugin/NvbloxMesh']
manager['Displays'][0]['Topic']['Value']='/in_hand_shader_test/mesh'
(out/'isolated_mesh.rviz').write_text(yaml.safe_dump(config,sort_keys=False))
resource.setrlimit(resource.RLIMIT_CORE,(0,0))
env=os.environ.copy();env['QT_QPA_PLATFORM']='xcb';env['LIBGL_ALWAYS_SOFTWARE']='1'
display=next(i for i in range(91,100) if not Path(f'/tmp/.X11-unix/X{i}').exists())
env['DISPLAY']=f':{display}';env.pop('XAUTHORITY',None)
rclpy.init();node=rclpy.create_node('in_hand_shader_regression')
pub=node.create_publisher(Mesh,'/in_hand_shader_test/mesh',1)
broadcaster=StaticTransformBroadcaster(node)
tf=TransformStamped();tf.header.frame_id='world';tf.child_frame_id='base_link';tf.transform.rotation.w=1.;broadcaster.sendTransform(tf)
report=dict(domain=os.environ.get('ROS_DOMAIN_ID'),fallback=os.environ.get('ISAAC_ROS_NVBLOX_PLUGIN_FORCE_FALLBACK_MATERIAL'),
            test='synthetic_mesh_offscreen',gazebo_started=False,arm_commands_sent=0,mesh_messages_sent=0)
child=None;xvfb=None;xlog=None
try:
    xlog=(out/'xvfb.log').open('w')
    xvfb=subprocess.Popen([str(root/'.test_runtime/usr/bin/Xvfb'),env['DISPLAY'],'-screen','0','1024x768x24','-nolisten','tcp','-ac'],stdout=xlog,stderr=subprocess.STDOUT)
    deadline=time.monotonic()+5.
    while not Path(f'/tmp/.X11-unix/X{display}').exists():
        if xvfb.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Virtual X server failed to start')
        time.sleep(.05)
    with (out/'rviz_offscreen.log').open('w') as log:
        child=subprocess.Popen(['rviz2','-d',str(out/'isolated_mesh.rviz'),'--ros-args','-p','use_sim_time:=false'],env=env,stdout=log,stderr=subprocess.STDOUT)
        began=time.monotonic();last=0.
        while time.monotonic()-began<10.:
            rclpy.spin_once(node,timeout_sec=.05)
            if child.poll() is not None:raise RuntimeError('RViz exited early: '+str(child.returncode))
            if pub.get_subscription_count() and time.monotonic()-last>.2:
                msg=Mesh();msg.header.frame_id='base_link';msg.header.stamp=node.get_clock().now().to_msg();msg.block_size_m=.16;msg.clear=True
                msg.block_indices=[Index3D(x=0,y=0,z=0)]
                block=MeshBlock();block.vertices=[Point32(x=0.,y=0.,z=0.),Point32(x=.1,y=0.,z=0.),Point32(x=0.,y=.1,z=0.)]
                block.triangles=[0,1,2];block.colors=[ColorRGBA(r=1.,g=0.,b=0.,a=1.) for _ in range(3)]
                msg.blocks=[block];pub.publish(msg);report['mesh_messages_sent']+=1;last=time.monotonic()
        if report['mesh_messages_sent']<3:raise RuntimeError('RViz did not subscribe to the test Mesh')
        report['survived_wall_seconds']=time.monotonic()-began
        child.send_signal(signal.SIGINT);child.wait(timeout=10)
        report.update(result='passed',exit_after_requested_shutdown=child.returncode)
except Exception as error:report.update(result='failed',reason=str(error))
finally:
    if child is not None and child.poll() is None:
        child.terminate()
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:child.kill();child.wait()
    if xvfb is not None and xvfb.poll() is None:
        xvfb.terminate();xvfb.wait(timeout=5)
    if xlog is not None:xlog.close()
    node.destroy_node();rclpy.try_shutdown()
    (out/'rviz_regression.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
raise SystemExit(0 if report['result']=='passed' else 1)
