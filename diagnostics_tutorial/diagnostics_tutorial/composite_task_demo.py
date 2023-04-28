import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import (
    CompositeDiagnosticTask,
    Updater,
    FunctionDiagnosticTask,
    DiagnosticStatusWrapper,
)
from rclpy.node import Node


def check_lower_bound(state: DiagnosticStatusWrapper):
    state.summary(DiagnosticStatus.OK, "lower ok")
    return state


def check_upper_bound(state: DiagnosticStatusWrapper):
    state.summary(DiagnosticStatus.OK, "upper ok")
    return state


class MyNode(Node):
    def __init__(self):
        node_name = "minimal"
        super().__init__(node_name)
        self.diagnostic_updater = Updater(self)
        lower = FunctionDiagnosticTask("lower check", check_lower_bound)
        upper = FunctionDiagnosticTask("upper check", check_upper_bound)
        comp_task = CompositeDiagnosticTask("range demo")
        comp_task.addTask(lower)
        comp_task.addTask(upper)
        self.diagnostic_updater.add(comp_task)
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
