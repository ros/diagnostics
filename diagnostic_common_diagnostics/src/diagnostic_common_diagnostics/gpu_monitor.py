#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, TNO IVS, Helmond, Netherlands
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the TNO IVS nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# \author Rein Appeldoorn

# This file is an edited copy of the cpu_monitor.py
# \author Roel Smallegoor

import rospy
from diagnostic_updater import DiagnosticTask, Updater
from diagnostic_msgs.msg import DiagnosticStatus

try:
    from pynvml import *
except:
    raise Exception, 'Needs python NVML module (https://pypi.python.org/pypi/nvidia-ml-py)'
import socket


class GpuTask(DiagnosticTask):
    deviceCount = 0

    def __init__(self, warning_percentage, warning_percentage_mem):
        DiagnosticTask.__init__(self, "GPU Information")
        self._warning_percentage = int(warning_percentage)
        self._warning_percentage_mem = int(warning_percentage_mem)
        try:
            nvmlInit()
            self.deviceCount = nvmlDeviceGetCount()
        except NVMLError as err:
            self.deviceCount = 0
            print(err)
            sys.exit(0)

    def run(self, stat):
        gpu_percentages = []
        for i in range(0, self.deviceCount):
            handle = nvmlDeviceGetHandleByIndex(i)
            try:
                util_rates = nvmlDeviceGetUtilizationRates(handle)
                gpu_percentages.append(util_rates.gpu)
                gpu_mem_percentages.append(util_rates.memory)
            except NVMLError as err:
                stat.summary(DiagnosticStatus.ERROR, err)
                return stat

        gpu_average = sum(gpu_percentages) / self.deviceCount
        gpu_mem_average = sum(gpu_mem_percentages) / self.deviceCount

        stat.add("GPU Load Average", gpu_average)
        stat.add("GPU Mem Load Average", gpu_mem_average)

        warn = False
        for idx, val in enumerate(gpu_percentages):
            stat.add("GPU {} Load".format(idx), "{}".format(val))
            if val > self._warning_percentage:
                warn = True

        warn_mem = False
        for idx, val in enumerate(gpu_mem_percentages):
            stat.add("GPU Memory {} Load".format(idx), "{}".format(val))
            if val > self._warning_percentage:
                warn_mem = True
            
        if warn:
            stat.summary(DiagnosticStatus.WARN, "At least one GPU exceeds %d percent\n GPU memory Average %.1f percent" % (self._warning_percentage_mem, gpu_mem_average) )
        elif warn_mem:
            stat.summary(DiagnosticStatus.WARN, "At least one GPU memory exceeds %d percent\n GPU Average %.1f percent" % (self._warning_percentage_mem, gpu_average) )
        elif warn and warn_mem:
            stat.summary(DiagnosticStatus.WARN, "At least one GPU exceeds %d percent\n At least one GPU memory exceeds %d percent" % (self._warning_percentage_mem, self._warning_percentage_mem) )
        else:
            stat.summary(DiagnosticStatus.OK, "GPU Average %.1f percent\n GPU memory Average %.1f percent" % (gpu_average, gpu_mem_average) )

        return stat


def main():
    hostname = socket.gethostname()
    rospy.init_node('gpu_monitor_%s' % hostname.replace("-", "_"))

    updater = Updater()
    updater.setHardwareID(hostname)
    updater.add(GpuTask(rospy.get_param("~warning_percentage", 90), rospy.get_param("~warning_percentage_mem", 90)))

    rate = rospy.Rate(rospy.get_param("~rate", 1))
    while not rospy.is_shutdown():
        rate.sleep()
        updater.update()


if __name__ == '__main__':
    main()
