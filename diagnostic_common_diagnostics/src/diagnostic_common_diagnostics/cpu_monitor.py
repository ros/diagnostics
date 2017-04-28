#!/usr/bin/env python

import rospy
from diagnostic_updater import DiagnosticTask, Updater
from diagnostic_msgs.msg import DiagnosticStatus

import psutil
import socket


class CpuTask(DiagnosticTask):
    def __init__(self, warning_percentage):
        DiagnosticTask.__init__(self, "CPU Information")
        self._warning_percentage = int(warning_percentage)

    def run(self, stat):
        cpu_percentages = psutil.cpu_percent(percpu=True)
        cpu_average = sum(cpu_percentages) / len(cpu_percentages)

        stat.add("CPU Load Average", cpu_average)

        warn = False
        for idx, val in enumerate(cpu_percentages):
            stat.add("CPU {} Load".format(idx), "{}".format(val))
            if val > self._warning_percentage:
                warn = True

        if warn:
            stat.summary(DiagnosticStatus.WARN, "At least one CPU exceeds %d percent" % self._warning_percentage)
        else:
            stat.summary(DiagnosticStatus.OK, "CPU Average %.1f percent" % cpu_average)

        return stat


def main():
    hostname = socket.gethostname()
    rospy.init_node('cpu_monitor_%s' % hostname.replace("-", "_"))

    updater = Updater()
    updater.setHardwareID(hostname)
    updater.add(CpuTask(rospy.get_param("~warning_percentage", 90)))

    rate = rospy.Rate(rospy.get_param("~rate", 1))
    while not rospy.is_shutdown():
        rate.sleep()
        updater.update()


if __name__ == '__main__':
    main()
