#!/usr/bin/env python3
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

# \author Ryan Friedman

from diagnostic_common_diagnostics import ntp_monitor
import pytest

def test_populate_invalid_host():
    rc,out,err = ntp_monitor.populate_ntpdate_output("foobar")
    assert rc != 0
    
def test_populated_valid_host():
    rc,out,err = ntp_monitor.populate_ntpdate_output(ntp_monitor.DEFAULT_NTP_HOSTNAME)
    assert rc == 0

def test_parse_valid_ouput():
    example_output = """
        server 165.140.142.118, stratum 2, offset -0.001559, delay 0.04453
        server 104.131.139.195, stratum 2, offset +0.000636, delay 0.05176
        server 38.229.54.9, stratum 2, offset -0.001806, delay 0.06880
        server 129.250.35.250, stratum 2, offset +0.006334, delay 0.04088
        23 Nov 17:30:41 ntpdate[1392]: adjust time server 129.250.35.250 offset +0.006334 sec
        """
    assert pytest.approx(ntp_monitor.parse_ntpdate_response_for_offset(example_output)) == -0.001559 * 1E6

def test_parse_empty_output():
    offset = ntp_monitor.parse_ntpdate_response_for_offset("")
    assert offset == None