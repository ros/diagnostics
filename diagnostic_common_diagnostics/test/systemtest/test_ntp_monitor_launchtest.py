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

import os

import ament_index_python

import launch

import launch_pytest
from launch_pytest.tools import process as process_tools

import launch_testing

import pytest


@pytest.fixture
def ntp_monitor_proc():
    # Launch a process to test
    return launch.actions.ExecuteProcess(
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


@launch_pytest.fixture
def launch_description(ntp_monitor_proc):
    return launch.LaunchDescription([
        ntp_monitor_proc,
        launch_testing.actions.ReadyToTest()
    ])


@pytest.mark.skip(reason='This test is not working yet')
@pytest.mark.launch(fixture=launch_description)
def test_read_stdout(ntp_monitor_proc, launch_context):
    """Check if 'ntp_monitor' was found in the stdout."""
    def validate_output(output):
        # this function can use assertions to validate the output or return a boolean.
        # pytest generates easier to understand failures when assertions are used.
        assert output.splitlines() == [
            'ntp_monitor'], 'process never printed ntp_monitor'
    process_tools.assert_output_sync(
        launch_context, ntp_monitor_proc, validate_output, timeout=5)

    def validate_output(output):
        return output == 'this will never happen'
    assert not process_tools.wait_for_output_sync(
        launch_context, ntp_monitor_proc, validate_output, timeout=0.1)
    yield
    # this is executed after launch service shutdown
    assert ntp_monitor_proc.return_code == 0
