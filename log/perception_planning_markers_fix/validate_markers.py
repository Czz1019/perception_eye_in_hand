import json
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('verify_marker_delivery')
counts = {'reliable': 0, 'best_effort': 0}
matched = dict.fromkeys(counts, 0)
latest_status = {}
sphere_count = 0
def callback(label, message):
    global sphere_count
    counts[label] += 1
    sphere_count = max(sphere_count, sum(m.ns == 'robot_collision' for m in message.markers))
    for marker in message.markers:
        p = marker.pose.position
        if (marker.ns == 'goal' and marker.header.frame_id == 'base_link'
                and max(abs(p.x-.5), abs(p.y-.5), abs(p.z-.45)) < 1e-6):
            matched[label] += 1

def status_cb(message):
    latest_status.clear()
    latest_status.update(json.loads(message.data))

for label, policy in [('reliable', ReliabilityPolicy.RELIABLE), ('best_effort', ReliabilityPolicy.BEST_EFFORT)]:
    node.create_subscription(MarkerArray, '/perception_planning/markers',
        lambda msg, label=label: callback(label, msg), QoSProfile(depth=1, reliability=policy))
node.create_subscription(String, '/perception_planning/status', status_cb, 1)
publisher = node.create_publisher(PoseStamped, '/target_pose', 1)
started = time.monotonic()
sent = False
while time.monotonic()-started < 90:
    rclpy.spin_once(node, timeout_sec=.1)
    if not sent and min(counts.values()) >= 3 and publisher.get_subscription_count():
        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.pose.position.x = target.pose.position.y = .5
        target.pose.position.z = .45
        target.pose.orientation.w = 1.
        publisher.publish(target)
        sent = True
    if min(matched.values()) >= 3:
        break
pubs = node.get_publishers_info_by_topic('/perception_planning/markers')
subs = node.get_subscriptions_info_by_topic('/perception_planning/markers')
rviz = [s for s in subs if s.node_name.startswith('rviz')]
checks = dict(reliable_received=matched['reliable']>=3, best_effort_received=matched['best_effort']>=3,
              publisher_reliable=bool(pubs) and all(p.qos_profile.reliability == ReliabilityPolicy.RELIABLE for p in pubs),
              rviz_reliable=bool(rviz) and all(s.qos_profile.reliability == ReliabilityPolicy.RELIABLE for s in rviz),
              robot_spheres=sphere_count==20, planner_no_error=bool(latest_status) and latest_status.get('error') is None)
result = dict(checks=checks, passed=all(checks.values()), counts=counts, goal_messages=matched,
              subscribers=[dict(node=s.node_name, reliability=str(s.qos_profile.reliability)) for s in subs],
              latest_status=latest_status)
Path('log/perception_planning_markers_fix/verification.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if result['passed'] else 1)
