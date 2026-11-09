"""Live map-update versus received-message rates; subscribes only to observed ESDF."""
import argparse
import json
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=0, help='0 keeps monitoring')
    parser.add_argument('--cloud-topic', default='/ur7e_nvblox/esdf_distance_band',
                        help='Use /ur7e_nvblox/esdf_observed for the legacy Python mapper')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = rclpy.create_node('ur7e_esdf_rates')
    received, stamps, points = [], set(), [0]
    started = last = time.monotonic()

    def cloud(msg):
        received.append(time.monotonic())
        stamps.add((msg.header.stamp.sec, msg.header.stamp.nanosec))
        points[0] = msg.width * msg.height

    def stats(msg):
        nonlocal last
        report = json.loads(msg.data)
        rates = report['rates_hz']
        now = time.monotonic()
        dt = max(now-last, 1e-6)
        latest = report.get('map_latest', {})
        input_key='input_queue_seen' if report.get('backend')=='official_ros_cpp' else 'depth_received'
        input_label='queued' if input_key=='input_queue_seen' else 'depth'
        completed = (f"{rates.get('esdf_completed', 0):5.1f}"
                     if report.get('interface_revision', 0) >= 4 else '  n/a')
        print(f"{input_label}={rates.get(input_key, 0):5.1f}  "
              f"fusion={rates.get('depth_integrated', 0):5.1f}  "
              f"ESDF_submit={rates.get('esdf_updated', 0):5.1f}  done={completed}  "
              f"export={rates.get('esdf_exported', 0):5.1f}  "
              f"received={len(received)/dt:5.1f}  "
              f"unique_image_stamps={len(stamps)/dt:5.1f} Hz  "
              f"points={points[0]}  map_version={latest.get('map_version', '-')}  "
              f"RGB={rates.get('color_integrated', 0):4.1f}Hz/{latest.get('color_state', '-')}  "
              f"error={report.get('error')}", flush=True)
        received.clear(); stamps.clear(); last = now

    node.create_subscription(PointCloud2, args.cloud_topic, cloud,
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(String, '/ur7e_nvblox/stats', stats, 2)
    print('Rates are wall-clock Hz. v4 done uses polled CUDA events; check dropped_gpu_measurements. '
          'unique_image_stamps counts distinct source images, not proven geometric changes.', flush=True)
    try:
        while rclpy.ok() and (args.seconds <= 0 or time.monotonic()-started < args.seconds):
            rclpy.spin_once(node, timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
