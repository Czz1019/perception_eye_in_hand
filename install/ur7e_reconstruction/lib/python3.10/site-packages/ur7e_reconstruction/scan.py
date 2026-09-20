"""Slow, feedback-checked Gazebo scanning using the existing position controller."""
import json
from pathlib import Path
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger, SetBool
from ament_index_python.packages import get_package_share_directory


class Scanner(Node):
    def __init__(self):
        super().__init__('reconstruction_scan')
        path = self.declare_parameter('views_file', str(Path(get_package_share_directory('ur7e_reconstruction')) / 'config/scan_views.yaml')).value
        config = yaml.safe_load(Path(path).read_text())
        self.names, self.views = config['joint_names'], config['views']
        self.speed = float(self.declare_parameter('max_speed_rad_s', 0.3).value)
        self.dwell = float(self.declare_parameter('dwell_sec', 5.0).value)
        if not np.isfinite(self.speed) or self.speed <= 0 or not np.isfinite(self.dwell) or self.dwell < 0:
            raise ValueError('Scan speed must be positive and dwell must be nonnegative')
        self.q, self.feedback_time, self.stats = None, 0, {}
        self.create_subscription(JointState, '/joint_states', self.joints, qos_profile_sensor_data)
        self.create_subscription(String, '/ur7e_reconstruction/stats', self.report, 5)
        self.command = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 5)
        self.save = self.create_client(Trigger, '/ur7e_reconstruction/save_mesh')
        self.integration = self.create_client(SetBool, '/ur7e_reconstruction/set_integration_enabled')
        self.records = []

    def joints(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(n in values for n in self.names):
            self.q = np.array([values[n] for n in self.names])
            self.feedback_time = time.monotonic()

    def report(self, msg):
        self.stats = json.loads(msg.data)

    def spin(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        if self.stats.get('error'):
            raise RuntimeError(self.stats['error'])

    def wait_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.spin()
            if predicate():
                return
        raise RuntimeError('Timeout: ' + description)

    def count(self):
        return self.stats.get('totals', {}).get('depth_integrated', 0)

    def set_integration(self, enabled):
        self.wait_until(self.integration.service_is_ready, 15, 'integration control service')
        future = self.integration.call_async(SetBool.Request(data=enabled))
        self.wait_until(future.done, 15, 'integration control reply')
        if not future.result().success:
            raise RuntimeError(future.result().message)

    def move(self, target):
        self.set_integration(False)
        start = self.q.copy()
        duration = max(3., 1.5 * float(np.max(np.abs(target-start))) / self.speed)
        # Use simulation time: slow Gazebo must not accelerate commanded joint motion.
        start_time = self.get_clock().now().nanoseconds / 1e9
        while rclpy.ok():
            self.spin()
            if time.monotonic() - self.feedback_time > 2.0:
                raise RuntimeError('Joint feedback stale; scan stopped')
            u = np.clip((self.get_clock().now().nanoseconds/1e9-start_time)/duration, 0, 1)
            command = start + (3*u*u-2*u*u*u)*(target-start)
            self.command.publish(Float64MultiArray(data=command.tolist()))
            if u >= 1:
                break
            time.sleep(.015)
        self.wait_until(lambda: np.max(np.abs(self.q-target)) < .008, 20, f'joint target {target.tolist()}')

    def run(self):
        self.wait_until(lambda: self.q is not None and self.count() > 2 and self.command.get_subscription_count() > 0,
                        120, 'camera reconstruction and controller ready')
        for view in self.views:
            self.get_logger().info('Moving to ' + view['name'])
            target = np.asarray(view['positions'])
            self.move(target)
            # Hold after reaching the target, then enable fusion explicitly.
            settled = time.monotonic() + .8
            while time.monotonic() < settled:
                self.spin()
                if np.max(np.abs(self.q-target)) >= .008:
                    raise RuntimeError('Joint target did not remain stable')
            self.set_integration(True)
            count = self.count()
            started = time.monotonic()
            self.wait_until(lambda: time.monotonic()-started >= self.dwell and self.count()-count >= 15,
                            90, 'stationary RGB-D integration')
            record = dict(name=view['name'], integrated_frames=self.count()-count,
                          joint_error_rad=float(np.max(np.abs(self.q-target))),
                          latest=self.stats.get('latest'))
            self.records.append(record)
            self.get_logger().info(json.dumps(record))
        self.wait_until(lambda: self.save.service_is_ready(), 10, 'mesh export service')
        future = self.save.call_async(Trigger.Request())
        self.wait_until(future.done, 30, 'mesh export')
        result = future.result()
        if not result.success:
            raise RuntimeError(result.message)
        output = Path(result.message).parent / 'scan_report.json'
        output.write_text(json.dumps(self.records, indent=2))
        self.get_logger().info(f'Scan complete: {result.message}; {output}')


def main(args=None):
    rclpy.init(args=args)
    node = Scanner()
    try:
        node.run()
    except BaseException:
        if node.q is not None and rclpy.ok():
            node.command.publish(Float64MultiArray(data=node.q.tolist()))
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
