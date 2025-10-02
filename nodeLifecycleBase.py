import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn


class SimpleLifecycleNode(Node):

    def __init__(self):
        super().__init__('simple_lifecycle_node')
        self.get_logger().info("Lifecycle node created")

    # Called when transitioning from "unconfigured" -> "inactive"
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring...")
        # Do setup here (e.g., create publishers, subscribers, timers)
        return TransitionCallbackReturn.SUCCESS

    # Called when transitioning from "inactive" -> "active"
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")
        # Activate publishers/subscribers/timers
        return TransitionCallbackReturn.SUCCESS

    # Called when transitioning from "active" -> "inactive"
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating...")
        # Deactivate publishers/subscribers/timers
        return TransitionCallbackReturn.SUCCESS

    # Called when transitioning from any state -> "finalized"
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")
        # Cleanup resources
        return TransitionCallbackReturn.SUCCESS

    # Called when shutting down
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.SingleThreadedExecutor()
    node = SimpleLifecycleNode()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#%