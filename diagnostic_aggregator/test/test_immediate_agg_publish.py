#!/usr/bin/env python
import os
import unittest

import rospy
import rostest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


class TestCustomAggregator(unittest.TestCase):
    def __init__(self, *args):
        super(TestCustomAggregator, self).__init__(*args)

        self.toplevel_rcvd = DiagnosticStatus.OK
        self.toplevel_msg_rcvd = 'Nothing received yet!'

        self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=4)
        self.diag_sub = rospy.Subscriber("/diagnostics_toplevel_state", DiagnosticStatus, self.toplevel_cb)

        # Make sure topics are connected
        rospy.sleep(rospy.Duration(1.0))

    def toplevel_cb(self, msg):
        self.toplevel_rcvd = msg.level
        self.toplevel_msg_rcvd = msg.message

    def publish_diagnostic(self, level, msg):
        error_msg = DiagnosticArray()
        self.error_status = DiagnosticStatus()
        self.error_status.level = level
        self.error_status.name = 'test: test-component'
        self.error_status.message = msg
        self.error_status.hardware_id = 'cpu'
        error_msg.status.append(self.error_status)
        error_msg.header.stamp = rospy.Time.now()
        self.diag_pub.publish(error_msg)

    def test_publish_immediate(self):
        # Start happy and wait for okay aggregation
        t0 = rospy.Time.now()
        while self.toplevel_rcvd != DiagnosticStatus.OK and rospy.Time.now() - t0 < rospy.Duration(3.0):
            self.publish_diagnostic(level=DiagnosticStatus.OK, msg='Nothing on the hand')
            rospy.sleep(rospy.Duration(0.5))

        # Get time, publish error and wait for new aggregated update
        t0 = rospy.Time.now()
        while self.toplevel_rcvd != DiagnosticStatus.ERROR and rospy.Time.now() - t0 < rospy.Duration(3.0):
            self.publish_diagnostic(level=DiagnosticStatus.ERROR, msg='This is the error message')
            rospy.sleep(rospy.Duration(0.1))
        t1 = rospy.Time.now()
        # Test time diff < 1 sec
        self.assertLess(t1 - t0, rospy.Duration(1.0), msg="Error message was not directly cascaded to aggregated topic")

    def test_message_in_toplevel(self):
        # Start happy and wait for okay aggregation
        t0 = rospy.Time.now()
        while self.toplevel_rcvd != DiagnosticStatus.OK and rospy.Time.now() - t0 < rospy.Duration(3.0):
            self.publish_diagnostic(level=DiagnosticStatus.OK, msg='Nothing on the hand')
            rospy.sleep(rospy.Duration(0.5))

        # Get time, publish error and wait for new aggregated update
        t0 = rospy.Time.now()
        error_msg = 'This is the error message'
        while self.toplevel_rcvd != DiagnosticStatus.ERROR and rospy.Time.now() - t0 < rospy.Duration(3.0):
            self.publish_diagnostic(level=DiagnosticStatus.ERROR, msg=error_msg)
            rospy.sleep(rospy.Duration(0.1))

        # Test for message in toplevel status
        self.assertTrue(self.toplevel_msg_rcvd == error_msg, msg='Error message not in toplevel state')


if __name__ == "__main__":
    rospy.init_node("test_custom_aggregator", anonymous=False)
    rostest.rosrun("diagnostic_aggregator", "test_custom_aggregator", TestCustomAggregator)
