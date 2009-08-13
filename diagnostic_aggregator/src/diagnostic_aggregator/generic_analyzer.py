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

PKG = 'diagnostic_aggregator'
import roslib; roslib.load_manifest(PKG)

import rospy
from xml.dom import minidom

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

stat_dict = { 0: 'OK', 1: 'Warning', 2: 'Error' }

class IncorrectNameError(Exception): pass

##\brief Stores DiagnosticStatus messages with update time
class DiagnosticItem:
    ##@param status DiagnosticStatus : First status message received
    def __init__(self, status):
        self._name = status.name
        self._level = status.level
        self._message = status.message
        self._hw_id = status.hardware_id
        self._values = status.values
        self._update_time = rospy.get_time()
        self._count = 0

    ## Update item with latest status
    ##@param status DiagnosticStatus : Latest message, must have same name as original
    def update(self, status):
        if self._name != status.name:
            raise IncorrectNameError

        self._level = status.level
        self._values = status.values
        self._message = status.message
        self._hw_id = status.hardware_id
        self._update_time = rospy.get_time()
        self._count += 1

    ##\brief Outputs diagnostics status
    ## Output item as DiagnosticStatus, sets level to stale if 
    ## timeout > 3.0 seconds
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

##\brief Analyzes all data for selected topics
## Initialized by XML snippet:
##\verbatim
## <analyzer type="GenericAnalyzer" pkg="diagnostic_aggregator" 
##           file="generic_analyzer" prefix="Motors" >
##  <topic startswith="EtherCAT" />
##  <topic startswith="Realtime" />
##  <topic contains="Power Node" />
##  <topic name="Mechanism Control" />
##</analyzer>
##\endverbatim
## The analyzer will group together selected topics according to the options 
## selected by the user. Users can specify status names by 'startswith', 
## 'contains', or 'name' for an exact name. 
## Analyzers are dynamically loaded by the aggregator.
## GenericAnalyzer stores, analyzes and outputs [ DiagnosticStatus ]
class GenericAnalyzer:
    ##@param xml_node minidom node : XML node to initialize analyzer
    ##@param prefix str : Prefix to be appended to all output 
    def __init__(self, xml_node, prefix):
        self._prefix = prefix

        self._status_names = {} # Store by last time

        self._startswith = []
        self._names = []
        self._contains = []

                
        for topic_node in xml_node.getElementsByTagName('topic'):
            topic_attrs = topic_node.attributes.keys()
            
            if 'startswith' in topic_attrs:
                starts_with = topic_node.attributes['startswith'].value
                self._startswith.append(starts_with)

            if 'name' in topic_attrs:
                self._names.append(str(topic_node.attributes['name'].value))

            if 'contains' in topic_attrs:
                self._contains.append(str(topic_node.attributes['contains'].value))

        
    ##\brief Analyze messages, returns processed array
    ## 
    ## Analyze messages and returns list of analyzed DiagnosticStatus msgs
    ##@param msgs dict : Dictionary of (msgs, count). Count++ if analyzed by this analyzer
    def analyze(self, msgs):
        status_array = []

        header_status = DiagnosticStatus()
        header_status.name = str(self._prefix)
        header_status.level = 0
        header_status.message = 'OK'
        header_status.values = []

        status_array.append(header_status)
        
        # Find messages to analyze
        msgs_to_analyze = self._select_msgs(msgs)
        
        self._update_status_names(msgs_to_analyze)

        # Process messages
        for name, item in self._status_names.iteritems():
            # Update level
            header_status.level = max(header_status.level, item._level)

            # Update value with last update time
            update_time = rospy.get_time() - item._update_time
            update_str = '%.3f' % update_time
            header_status.values.append(KeyValue(name, update_str))
            stale = update_time > 3.0 ##@todo Make this parameter
            
            # Output status message
            status_array.append(item.to_status_msg(self._prefix, stale))
    
        return status_array

    ##\brief Returns messages to analyze based on topic name
    def _select_msgs(self, msgs):
        to_analyze = []
        for msg, value in msgs.iteritems():
            for startswith in self._startswith:
                if msg.name.startswith(startswith):
                    to_analyze.append(msg)
                    msgs[msg] += 1
                    continue

            for name in self._names:
                if msg.name == name:
                    to_analyze.append(msg)
                    msgs[msg] += 1
                    continue

            for contain in self._contains:
                if msg.name.find(contain) > -1:
                    to_analyze.append(msg)
                    msgs[msg] += 1
                    continue

        return to_analyze

    ##\brief Updates stored array of status messages 
    ##@param msgs list : List of DiagnosticStatus messages
    def _update_status_names(self, msgs):
        for msg in msgs:
            if not self._status_names.has_key(msg.name):
                self._status_names[msg.name] = DiagnosticItem(msg)
            self._status_names[msg.name].update(msg)


        
        

        
                
