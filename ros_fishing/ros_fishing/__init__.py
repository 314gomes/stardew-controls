from . import fishing
import rclpy
import rclpy.executors

def main():
    rclpy.init()
    node = fishing.FishingNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()