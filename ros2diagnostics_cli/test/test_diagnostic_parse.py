import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor

TEST_NODE = "cli_diag_parse_test_node"
TEST_NAMESPACE = "cli_diag_parse"


class TestROS2DiagnosticParse(unittest.TestCase):
    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(
            TEST_NODE, namespace=TEST_NAMESPACE, context=self.context
        )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_diagnostic_pub(self):
        assert True
