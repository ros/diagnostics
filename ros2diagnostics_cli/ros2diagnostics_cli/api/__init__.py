import rclpy
from diagnostic_msgs.msg import DiagnosticArray

TOPIC_DIAGNOSTICS = "/diagnostics"


def diagnostic_handler(msg: DiagnosticArray) -> None:
    print(msg)


def get_hello_world():
    rclpy.init()
    node = rclpy.create_node("ros2diagnostics_cli_filter")
    node.create_subscription(DiagnosticArray, TOPIC_DIAGNOSTICS, diagnostic_handler, 10)
    rclpy.spin_once(node)


def get_hello_world_leet():
    return "He110, R0S 2 W04ld!"
