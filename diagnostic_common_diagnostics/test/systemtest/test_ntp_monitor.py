#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2023, Robert Bosch GmbH
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
#  * Neither the name of the Willow Garage nor the names of its
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

import launch_testing
import launch
import unittest
import pytest
import os

import ament_index_python


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        # launch_ros.actions.Node(
        #     package='diagnostic_common_diagnostics',
        #     node_executable='ntp_monitor',
        #     output='screen',
        #     arguments=['--ntp-hostname', 'localhost']
        # ),
        launch.actions.ExecuteProcess(
            cmd=[
                os.path.join(
                    ament_index_python.get_package_prefix(
                        'diagnostic_common_diagnostics'),
                    'lib',
                    'diagnostic_common_diagnostics',
                    'ntp_monitor.py'
                ),
            ],
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestNTPMonitor(unittest.TestCase):

    def test_process_starts(self, proc_output):
        """Test that the process starts and exits normally."""
        proc_output.assertWaitFor('NTPMonitor: Starting up', timeout=5)
        proc_output.assertNotWaitFor('FileNotFoundError', timeout=5)
