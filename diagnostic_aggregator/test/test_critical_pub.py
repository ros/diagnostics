import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    # Launch the aggregator
    parameters = [{'analyzers.test.type': 'diagnostic_aggregator/GenericAnalyzer'},
                  {'analyzers.test.path': 'Test'},
                  {'analyzers.test.contains': ['test']},
                  {'critical': True}]

    aggregator_cmd = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        parameters=parameters)

    ld = LaunchDescription()

    # Launch the node
    ld.add_action(aggregator_cmd)
    ld.add_action(ReadyToTest())
    return ld


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
        self.node = rclpy.create_node('test_node')
        self.publisher = self.node.create_publisher(DiagnosticArray, '/diagnostics', 1)
        self.subscriber = self.node.create_subscription(
            DiagnosticStatus,
            '/diagnostics_toplevel_state',
            lambda msg: self.received_state.append(msg.level),
            1)
        self.received_state = []

    def tearDown(self):
        self.node.destroy_node()

    def publish_message(self, level):
        msg = DiagnosticArray()
        msg.status.append(DiagnosticStatus())
        msg.status[0].level = level
        msg.status[0].name = 'test status'
        while msg.status[0].level not in self.received_state:
            self.received_state.clear()
            self.publisher.publish(msg)
            rclpy.spin_once(self.node)
        return self.node.get_clock().now()

    def critical_publisher_test(
        self, initial_state=DiagnosticStatus.OK, new_state=DiagnosticStatus.ERROR
    ):
        # Publish the ok message and wait till the toplevel state is received
        time_0 = self.publish_message(initial_state)

        assert (self.received_state[0] == initial_state), \
            ('Received state is not the same as the sent state:'
                + f"'{self.received_state[0]}' != '{initial_state}'")
        self.received_state.clear()

        # Publish the ok message and expect the toplevel state after 1 second period
        time_1 = self.publish_message(initial_state)
        assert (time_1 - time_0 > rclpy.duration.Duration(seconds=0.99)), \
            'OK message received too early'
        assert (self.received_state[0] == initial_state), \
            ('Received state is not the same as the sent state:'
                + f"'{self.received_state[0]}' != '{initial_state}'")
        self.received_state.clear()

        # Publish the message and expect the critical error message immediately
        time_2 = self.publish_message(new_state)

        assert (time_2 - time_1 < rclpy.duration.Duration(seconds=0.1)), \
            'Critical error message not received within 0.1 second'
        assert (self.received_state[0] == new_state), \
            ('Received state is not the same as the sent state:'
                + f"'{self.received_state[0]}' != '{new_state}'")
        self.received_state.clear()

        # Next error message should be sent at standard 1 second rate
        time_3 = self.publish_message(new_state)

        assert (time_3 - time_1 > rclpy.duration.Duration(seconds=0.99)), \
            'Periodic error message received too early'
        assert (self.received_state[0] == new_state), \
            ('Received state is not the same as the sent state:'
                + f"'{self.received_state[0]}' != '{new_state}'")

    def test_critical_publisher_ok_error(self):
        self.critical_publisher_test(
            initial_state=DiagnosticStatus.OK, new_state=DiagnosticStatus.ERROR
        )

    def test_critical_publisher_ok_warn(self):
        self.critical_publisher_test(
            initial_state=DiagnosticStatus.OK, new_state=DiagnosticStatus.WARN
        )

    def test_critical_publisher_warn_error(self):
        self.critical_publisher_test(
            initial_state=DiagnosticStatus.WARN, new_state=DiagnosticStatus.ERROR
        )
