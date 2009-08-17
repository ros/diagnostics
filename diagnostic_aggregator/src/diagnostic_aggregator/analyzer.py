#! /usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#

# Author: Kevin Watts

##\brief Provides useful classes for analyzers to use

PKG = 'diagnostic_aggregator'
import roslib; roslib.load_manifest(PKG)

import rospy

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

stat_dict = { 0: 'OK', 1: 'Warning', 2: 'Error' }

##\brief Raises for incorrect name in DiagnosticStatus
class IncorrectNameError(Exception): pass

##\brief Raises for parameters not found in analyzer
class ParameterNotFoundError(Exception) : pass

##\brief Stores DiagnosticStatus messages with update time
##
## Useful class for storing DiagnosticStatus messages, tracking stale messages
class DiagnosticItem:
    ##\param status DiagnosticStatus : First status message received
    def __init__(self, status):
        self._name = status.name
        self._level = status.level
        self._message = status.message
        self._hw_id = status.hardware_id
        self._values = status.values
        self._update_time = rospy.get_time()
        self._count = 0

    ##\brief Update item with latest status
    ##
    ##\param status DiagnosticStatus : Latest message, must have same name as original
    def update(self, status):
        if self._name != status.name:
            raise IncorrectNameError

        self._level = status.level
        self._values = status.values
        self._message = status.message
        self._hw_id = status.hardware_id
        self._update_time = rospy.get_time()
        self._count += 1

    ##\brief Outputs item as DiagnosticStatus message
    ##
    ## Output item as DiagnosticStatus, sets level to stale if stale true
    ##\param prefix str : Prepended to status name
    ##\param stale bool : If true, set status.level to 3 (Stale)
    def to_status_msg(self, prefix, stale):
        status = DiagnosticStatus()
        status.name = str(prefix + '/' + self._name.replace('/', ''))
        status.level = self._level
        status.hardware_id = self._hw_id
        status.message = self._message
        status.values = self._values

        if stale:
            status.level = 3

        return status
