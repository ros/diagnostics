import rclpy
from rclpy.node import Node
from diagnostic_updater import DiagnosticTask, DiagnosticStatusWrapper, Updater
from diagnostic_msgs.msg import DiagnosticStatus


class DummyTask(DiagnosticTask):
    def run(self, stat: DiagnosticStatusWrapper):
        stat.summary(DiagnosticStatus.WARN, "DummyTask demo")
        return stat


class MyNode(Node):
    def __init__(self):
        node_name = "minimal"
        super().__init__(node_name)
        self.diagnostic_updater = Updater(self)
        self.diagnostic_updater.setHardwareID("demo hw")
        self.diagnostic_updater.add(DummyTask("dummy_task"))
        self.create_timer(1.0, self.__timer_handler)
        self.get_logger().info("Hello ROS2")

    def __timer_handler(self):
        self.diagnostic_updater.update()


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("User exit")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
