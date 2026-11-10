import sys,json,time
from pathlib import Path
import numpy as np
import rclpy
from nvblox_msgs.srv import EsdfAndGradients
p=Path(sys.argv[1]);rclpy.init();n=rclpy.create_node('official_esdf_snapshot');c=n.create_client(EsdfAndGradients,'/nvblox_node/get_esdf_and_gradient')
if not c.wait_for_service(timeout_sec=15):raise RuntimeError('ESDF service absent')
r=EsdfAndGradients.Request();r.update_esdf=True;r.visualize_esdf=False;r.use_aabb=True;r.frame_id='base_link';r.aabb_min_m.x=.16;r.aabb_min_m.y=-.5;r.aabb_min_m.z=0.;r.aabb_size_m.x=.84;r.aabb_size_m.y=1.;r.aabb_size_m.z=.4
f=c.call_async(r);rclpy.spin_until_future_complete(n,f,timeout_sec=30)
if not f.done() or f.result() is None or not f.result().success:raise RuntimeError('ESDF service failed')
a=f.result();dims=[d.size for d in a.esdf_and_gradients.layout.dim];meta={'origin':[a.origin_m.x,a.origin_m.y,a.origin_m.z],'voxel_size':a.voxel_size_m,'dims':dims,'labels':[d.label for d in a.esdf_and_gradients.layout.dim],'strides':[d.stride for d in a.esdf_and_gradients.layout.dim]};data=np.asarray(a.esdf_and_gradients.data,np.float32);np.save(p/'esdf_response.npy',data);(p/'esdf_response.json').write_text(json.dumps(meta,indent=2));print(meta,'elements',len(data));n.destroy_node();rclpy.shutdown()
