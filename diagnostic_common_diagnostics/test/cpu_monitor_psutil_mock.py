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


import sys
from argparse import ArgumentParser


class PSUtilMock:
    CPU_PERCENTAGE = 0

    def __init__(self):
        pass

    @staticmethod
    def cpu_percent(percpu=False):
        if percpu:
            return [PSUtilMock.CPU_PERCENTAGE] * 4


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--percentage', default=50, type=int)
    args = parser.parse_known_args()[0]

    sys.modules['psutil'] = PSUtilMock
    from diagnostic_common_diagnostics import cpu_monitor

    PSUtilMock.CPU_PERCENTAGE = args.percentage
    cpu_monitor.main()
