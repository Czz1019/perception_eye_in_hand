"""Measured collision cover and TCP trace in RViz."""
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray

def publish(node,q):
    centers=node.g.spheres(q);items=[]
    for i,(p,r) in enumerate(zip(centers,node.g.radii)):
        m=Marker();m.header.frame_id='base_link';m.ns='body';m.id=i;m.type=Marker.SPHERE
        m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,p);m.pose.orientation.w=1.
        m.scale.x=m.scale.y=m.scale.z=float(r*2);m.color.a=.22
        m.color.b=1.;m.color.g=.6 if i<16 else .1;m.color.r=.1
        items.append(m)
    tcp=node.g.fk(q)['tool0'][:3,3]
    if not node.tcp_trace or sum((a-b)**2 for a,b in zip(tcp,node.tcp_trace[-1]))>1e-6:
        node.tcp_trace.append(tcp.copy());node.tcp_trace=node.tcp_trace[-6000:]
    m=Marker();m.header.frame_id='base_link';m.ns='actual_tcp';m.id=0;m.type=Marker.LINE_STRIP
    m.pose.orientation.w=1.;m.scale.x=.003;m.color.a=1.;m.color.g=1.;m.color.r=.1
    m.points=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in node.tcp_trace];items.append(m)
    if node.goal is not None:
        m=Marker();m.header.frame_id='base_link';m.ns='goal';m.id=0;m.type=Marker.SPHERE
        m.pose.position=Point(x=float(node.goal[0]),y=float(node.goal[1]),z=float(node.goal[2]))
        m.pose.orientation.w=1.;m.scale.x=m.scale.y=m.scale.z=.025;m.color.r=1.;m.color.a=1.;items.append(m)
    node.markers.publish(MarkerArray(markers=items))
