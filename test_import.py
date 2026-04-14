from newton_ros import NewtonRosBridge
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor,SingleThreadedExecutor
import newton
import threading
import warp as wp

def spin_node(node):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()


def main(args=None):
    rclpy.init(args=args)

    warp_lock = threading.Lock()

    default_ros_node = Node("newton_ros_bridge_node")

    newton_ros_bridge = NewtonRosBridge(
        default_ros_node,
        "src/configs/ackerman_demo.yaml",
        enable_simulation_interfaces=True,
    )
    newton_ros_bridge.build()

    # Warmup
    for _ in range(30):
        with warp_lock:
            newton_ros_bridge.step()
    
    spin_threads = []

    # One executor + thread per node
    for node in newton_ros_bridge.all_nodes_to_spin:
        t = threading.Thread(
            target=spin_node,
            args=(node,),
            daemon=True,
        )
        t.start()
        spin_threads.append(t)

    try:
        while rclpy.ok():
            newton_ros_bridge.step()

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            rclpy.shutdown()

        # Optional: join threads briefly (optional)
        for t in spin_threads:
            t.join(timeout=0.1)

        del newton_ros_bridge


if __name__ == "__main__":
    main()
