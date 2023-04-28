import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, Updater
from rclpy.node import Node


class DummyClass:
    def produce_diagnostics(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        stat.summary(DiagnosticStatus.WARN, "demo class status")
        stat.add("dummy data", "2000")
        return stat

def dummy_diagnostics(stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
    stat.summary(DiagnosticStatus.OK, "dummy_diagnostics function")
    return stat

class MyNode(Node):
    def __init__(self):
        node_name = "minimal"
        super().__init__(node_name)
        self.diagnostic_updater = Updater(self)
        dc = DummyClass()
        self.diagnostic_updater.add("method updater", dc.produce_diagnostics)
        self.diagnostic_updater.add("function update", dummy_diagnostics)
        self.create_timer(1.0, self.__timer_handler)
        self.get_logger().info("Hello ROS2")

    def __timer_handler(self):
        self.diagnostic_updater.update()


def main():
    rclpy.init()
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
