import rclpy
from rclpy.node import Node
from diagnostic_updater import HeaderlessTopicDiagnostic, FrequencyStatusParam, Updater


class MyNode(Node):
    def __init__(self):
        node_name = "frequency_diagnostics"
        super().__init__(node_name)

        self.diagnostic_updater = Updater(self)
        freq_bound = {"min": 0.5, "max": 2}
        self.pub_freq = HeaderlessTopicDiagnostic(
            "topic1", self.diagnostic_updater, FrequencyStatusParam(freq_bound)
        )

        self.create_timer(3.0, self.__timer_handler)
        self.get_logger().info("Hello ROS2")

    def __timer_handler(self):
        self.pub_freq.tick()
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
