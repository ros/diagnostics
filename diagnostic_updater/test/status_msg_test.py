import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from diagnostic_updater import Updater
import rclpy


class TestProcessOutput(unittest.TestCase):

    @ classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @ classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('listener_node')
        self.subscriber = self.node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            lambda msg: self.received_status.append(msg.status[0]),
            1)
        self.received_status = []
        self.updater_node_name = 'updater_node'

    def tearDown(self):
        self.node.destroy_node()

    def update_diagnostics(self, stat):
        stat.summary(DiagnosticStatus.OK, 'test')
        return stat

    def test_name(self):
        updater_node = rclpy.create_node(self.updater_node_name)
        updater = Updater(updater_node)
        updater.setHardwareID('hardware_test')
        updater.add('test check', self.update_diagnostics)
        updater_node.create_timer(0.1, lambda: updater.update())

        while len(self.received_status) < 1:
            rclpy.spin_once(updater_node)
            rclpy.spin_once(self.node)

        self.assertEqual(self.received_status[0].name, self.updater_node_name + ': test check')
        self.assertEqual(self.received_status[0].message, 'test')
        self.assertEqual(self.received_status[0].level, DiagnosticStatus.OK)
        self.assertEqual(self.received_status[0].hardware_id, 'hardware_test')
        updater_node.destroy_node()

    def test_fully_qualified_name(self):
        param = [rclpy.Parameter('diagnostic_updater.use_fqn', rclpy.Parameter.Type.BOOL, True)]
        updater_node = rclpy.create_node(node_name=self.updater_node_name,
                                         namespace='test_namespace', parameter_overrides=param,
                                         automatically_declare_parameters_from_overrides=True)
        updater = Updater(updater_node)
        updater.setHardwareID('hardware_test')
        updater.add('test_check', self.update_diagnostics)
        updater_node.create_timer(0.1, lambda: updater.update())

        while len(self.received_status) < 1:
            rclpy.spin_once(updater_node)
            rclpy.spin_once(self.node)

        self.assertEqual(self.received_status[0].name,
                         '/test_namespace/updater_node: test_check')
        self.assertEqual(self.received_status[0].message, 'test')
        self.assertEqual(self.received_status[0].level, DiagnosticStatus.OK)
        self.assertEqual(self.received_status[0].hardware_id, 'hardware_test')
