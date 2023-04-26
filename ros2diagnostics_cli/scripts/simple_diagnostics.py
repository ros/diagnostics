import rclpy
from rclpy.node import Node
from diagnostic_updater import (DiagnosticTask,
    Updater,
    DiagnosticStatusWrapper)
from diagnostic_msgs.msg import DiagnosticStatus


class DemoStatus(DiagnosticTask):
    def __init__(self, name):
        super().__init__(name)

    def run(self, stat: DiagnosticStatusWrapper):
        stat.summary(DiagnosticStatus.WARN, "running")
        stat.add("key1", "val1")
        stat.add("key2", "val2")
        return stat

class Demo1Status(DiagnosticTask):
    def __init__(self, name):
        super().__init__(name)

    def run(self, stat: DiagnosticStatusWrapper):
        stat.summary(DiagnosticStatus.ERROR, "bad")
        return stat

class MyNode(Node):
    def __init__(self):
        node_name = "diagnostic_simple"
        super().__init__(node_name)
        self.diag_update = Updater(self)
        self.diag_update.add(DemoStatus("DemoTask"))
        self.diag_update.add(Demo1Status("DemoTask2"))
        self.get_logger().info("Hello ROS2")


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
