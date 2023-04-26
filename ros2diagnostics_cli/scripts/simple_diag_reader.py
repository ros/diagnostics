import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from diagnostic_msgs.msg import DiagnosticArray
from rosidl_runtime_py import message_to_csv

class MyNode(Node):
    def __init__(self):
        node_name="simple_diag_reader"
        super().__init__(node_name)
        self.create_subscription(DiagnosticArray, "/diagnostics", self.__handler, qos_profile=qos_profile_system_default)
        self.get_logger().info("Hello diagnostics parser")

    def __handler(self, msg: DiagnosticArray):
        for status in msg.status:
            to_print = message_to_csv(
                status)

            print(to_print)


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

if __name__ == '__main__':
    main()