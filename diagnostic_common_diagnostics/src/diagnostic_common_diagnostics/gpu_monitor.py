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
# TODO

import rospy
from diagnostic_updater import DiagnosticTask, Updater
from diagnostic_msgs.msg import DiagnosticStatus

#import psutil
import socket
from pynvml import *


class GpuTask(DiagnosticTask):
    def __init__(self, warning_percentage):
        DiagnosticTask.__init__(self, "GPU Information")
        self._warning_percentage = int(warning_percentage)
        try:
            nvmlInit()
            deviceCount = nvmlDeviceGetCount()
        except NVMLError as err:
            #TODO #strResult += 'nvidia_smi.py: ' + err.__str__() + '\n'
            deviceCout = 0

    def run(self, stat):
            for i in range(0, deviceCount):
                handle = nvmlDeviceGetHandleByIndex(i)
                try:
                    util = nvmlDeviceGetUtilizationRates(handle)
                    gpu_util = str(util.gpu)
                    mem_util = str(util.memory)
                except NVMLError as err:
                    error = handleError(err)
                    gpu_util = error
                    mem_util = error

                try:
                    memInfo = nvmlDeviceGetMemoryInfo(handle)
                    mem_total = str(memInfo.total / 1024 / 1024) + ' MB'
                    mem_used = str(memInfo.used / 1024 / 1024) + ' MB'
                    mem_free = str(memInfo.free / 1024 / 1024) + ' MB'
                except NVMLError as err:
                    error = handleError(err)
                    mem_total = error
                    mem_used = error
                    mem_free = error

        gpu_percentages = 0 #psutil.gpu_percent(pergpu=True)
        gpu_average = sum(gpu_percentages) / len(gpu_percentages)

        stat.add("GPU Load Average", gpu_average)

        warn = False
        for idx, val in enumerate(gpu_percentages):
            stat.add("GPU {} Load".format(idx), "{}".format(val))
            if val > self._warning_percentage:
                warn = True

        if warn:
            stat.summary(DiagnosticStatus.WARN, "At least one GPU exceeds %d percent" % self._warning_percentage)
        else:
            stat.summary(DiagnosticStatus.OK, "GPU Average %.1f percent" % gpu_average)

        return stat


def main():
    hostname = socket.gethostname()
    rospy.init_node('gpu_monitor_%s' % hostname.replace("-", "_"))

    updater = Updater()
    updater.setHardwareID(hostname)
    updater.add(GpuTask(rospy.get_param("~warning_percentage", 90)))

    rate = rospy.Rate(rospy.get_param("~rate", 1))
    while not rospy.is_shutdown():
        rate.sleep()
        updater.update()


if __name__ == '__main__':
    main()
