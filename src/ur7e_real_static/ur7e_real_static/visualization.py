"""Subscriber-gated, latest-only visualization; never part of the motion gate."""
import time
import numpy as np
from realtime_nvblox.ros_publishers import esdf_cloud


def worker(node):
    published={'planning':-1,'surface':-1};next_due=0.
    while not node.stop_event.wait(.05):
        if time.monotonic()<next_due:continue
        sequence,item=node.visuals.latest()
        if item is None:continue
        epoch,points,distance,stamp,raw,unknown=item
        with node.lock:
            if epoch!=node.epoch:continue
        try:
            for kind,pub in [('planning',node.cloud_pub),('surface',node.surface_pub)]:
                # A retained latest item also serves subscribers joining after freeze.
                if published[kind]==sequence or not pub.get_subscription_count():continue
                start=time.perf_counter()
                if kind=='planning':
                    visible=np.isfinite(distance)&(distance>0)
                    cloud=esdf_cloud(points[visible],distance[visible],'base_link',int(stamp*1e9),.15,unknown_distance_m=unknown)
                else:
                    # Reloaded maps contain no native surface field: clear old display.
                    p=points if raw is not None else np.empty((0,3))
                    d=raw if raw is not None else np.empty(0)
                    cloud=esdf_cloud(p,d,'base_link',int(stamp*1e9),.05,unknown_distance_m=unknown)
                with node.lock:
                    if epoch!=node.epoch:break
                pub.publish(cloud);published[kind]=sequence
                node.capture_stats.inc('visualizations_published')
                node.capture_stats.add_latency('visualization',(time.perf_counter()-start)*1000)
            next_due=time.monotonic()+1/node.c['visualization_hz']
        except Exception as exc:
            # Rendering failure must neither refresh a map nor disable a motion watchdog.
            node.get_logger().warning('ESDF visualization failed: '+str(exc),throttle_duration_sec=5.)
