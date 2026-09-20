import argparse
import os


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['mapper', 'planner', 'combined'])
    args, ros_args = parser.parse_known_args()
    os.environ.setdefault('OMP_NUM_THREADS', '2')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
    rclpy.init(args=ros_args)
    if args.mode == 'combined':
        from .mapper import PlanningMapper
        from .planner import Planner
        mapper, planner = PlanningMapper(), Planner()
        mapper.map_consumer = planner.on_map
        nodes = [planner, mapper]
    elif args.mode == 'mapper':
        from .mapper import PlanningMapper
        nodes = [PlanningMapper()]
    else:
        from .planner import Planner
        nodes = [Planner()]
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # SIGINT may invalidate the ROS context while Humble is taking a DDS
        # message, before it raises ExternalShutdownException. Only suppress
        # that shutdown race; errors while the context is alive must propagate.
        if rclpy.ok():
            raise
    finally:
        for node in nodes:
            node.close()
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
