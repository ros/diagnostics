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
    raise Exception, 'Python NVML module required (https://pypi.python.org/pypi/nvidia-ml-py)'
import socket


class GpuTask(DiagnosticTask):
    def __init__(self, handle, warning_percentage, warning_percentage_mem, warning_percentage_mem_usage):
        DiagnosticTask.__init__(self, "GPU Information")
        self._handle = handle
        self._warning_percentage = int(warning_percentage)
        self._warning_percentage_mem = int(warning_percentage_mem)
        self._warning_percentage_mem_usage = int(warning_percentage_mem_usage)

    def run(self, stat):
        gpu_percentages = []
        gpu_mem_percentages = []
        gpu_mem_usage_percentages = []
        try:
            util_rates = nvmlDeviceGetUtilizationRates(self._handle)
            mem_info = nvmlDeviceGetMemoryInfo(self._handle)
            gpu_percentage = util_rates.gpu
            gpu_mem_percentage = util_rates.memory
            gpu_mem_usage_percentage = int(mem_info.used*100.0/mem_info.total)
        except NVMLError as err:
            stat.summary(DiagnosticStatus.ERROR, err)
            return stat

        warn = False
        stat.add("Load [%]", gpu_percentage)
        if gpu_percentage > self._warning_percentage:
            warn = True

        warn_mem = False
        stat.add("Memory load [%]", gpu_mem_percentage)
        if gpu_mem_percentage > self._warning_percentage_mem:
            warn_mem = True

        warn_mem_usage = False
        stat.add("Memory usage [%]", gpu_mem_usage_percentage)
        if gpu_mem_usage_percentage > self._warning_percentage_mem_usage:
            warn_mem_usage = True
            
        if warn:
            stat.summary(DiagnosticStatus.WARN, "GPU exceeds %d percent" % (self._warning_percentage_mem) )
        elif warn_mem:
            stat.summary(DiagnosticStatus.WARN, "GPU memory exceeds %d percent" % (self._warning_percentage_mem) )
        elif warn_mem_usage:
            stat.summary(DiagnosticStatus.WARN, "GPU memory usage exceeds %d percent" % (self._warning_percentage_mem_usage) )
        else:
            stat.summary(DiagnosticStatus.OK, "GPU Status OK")
			
        return stat


def main():
    hostname = socket.gethostname()
    rospy.init_node('gpu_monitor_%s' % hostname.replace("-", "_"))

    nvmlInit()
    try:
        device_count = nvmlDeviceGetCount()
    except NVMLError as err:
        print(err)
        sys.exit(0)

    if (device_count == 0):
        print("No GPU device found")
        sys.exit(0)

    updaters = []
    for i in range(0, device_count):
        try:
            handle = nvmlDeviceGetHandleByIndex(i)
            name = nvmlDeviceGetName(handle)
        except NVMLError as err:
            print(err)
            sys.exit(0)
        updater = Updater()
        updater.setHardwareID(name)
        updater.add(GpuTask(handle, rospy.get_param("~warning_percentage", 90), rospy.get_param("~warning_percentage_mem", 90), rospy.get_param("~warning_percentage_mem_usage", 90)))
        updaters.append(updater)

    rate = rospy.Rate(rospy.get_param("~rate", 1))
    while not rospy.is_shutdown():
        rate.sleep()
        for i in range(0, device_count):
            updaters[i].update()


if __name__ == '__main__':
    main()
