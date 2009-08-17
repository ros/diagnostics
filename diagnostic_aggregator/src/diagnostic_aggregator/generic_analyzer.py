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

##\brief GenericAnalyzer is a basic analyzer.

PKG = 'diagnostic_aggregator'
import roslib; roslib.load_manifest(PKG)

import rospy
from xml.dom import minidom

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

stat_dict = { 0: 'OK', 1: 'Warning', 2: 'Error' }

from diagnostic_aggregator.analyzer import *

##\brief Analyzes all data for selected topics. Default analyzer.
##
## GenericAnalyzer is initialized by XML snippet:
##\verbatim
## <analyzer type="GenericAnalyzer" pkg="diagnostic_aggregator" 
##           file="generic_analyzer" prefix="Motors" 
##           expected="my_expected" >
##  <topic startswith="EtherCAT" />
##  <topic startswith="Realtime" />
##  <topic contains="Power Node" />
##  <topic name="Mechanism Control" />
##</analyzer>
##\endverbatim
## The analyzer will group together selected topics according to the options 
## selected by the user. Users can specify status names by 'startswith', 
## 'contains', or 'name' for an exact name. 
## 
## Any expected names can be set with the 'expected' parameter name. The analyzer
## will read the '~my_expected' parameter from the parameter server and compare it
## to the list of topics received. Any topics in the expected list do not need to be
## set with the 'topic' tag.
## 
## Analyzers are dynamically loaded by the aggregator.
## GenericAnalyzer stores, analyzes and outputs [ DiagnosticStatus ]
class GenericAnalyzer:
    ##\param xml_node minidom node : XML node to initialize analyzer
    ##\param prefix str : Prefix to be appended to all output 
    ##\param other bool (optional) : Handles remaining messages if true
    def __init__(self, xml_node, prefix, other = False):
        self._prefix = prefix

        self._status_names = {} # Store by last time

        self._other = other
        if self._other:
            return
        
        self._startswith = []
        self._names = []
        self._contains = []

        self._expected = []

        # Any names that are expected are automatically added to the 
        # names list, with error state and 'Missing' message
        if xml_node.attributes.has_key('expected'):
            expected_param = xml_node.attributes['expected'].value
            params = rospy.get_param('~%s' % expected_param, None)
            if not params:
                raise ParameterNotFoundError
            for name in params:
                self._expected.append(name)
                exp_stat = DiagnosticStatus()
                exp_stat.name = name
                exp_stat.level = 2
                exp_stat.message = 'Missing'
                self._status_names[name] = DiagnosticItem(exp_stat)

        # Load remaining topic nodes
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
    ##\param msgs dict{DiagnosticStatus, count} : count++ if analyzed by this analyzer
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
        all_stale = True
        for name, item in self._status_names.iteritems():
            # Update stale
            update_time = rospy.get_time() - item._update_time
            stale = update_time > 3.0 ##@todo Make this parameter
            all_stale = stale and all_stale

            # Update level
            header_status.level = max(header_status.level, item._level)
            level_str = stat_dict[item._level]

            # Automatically error header if it's stale
            if stale:
                header_status.level = 2
                level_str = 'Stale'

            # Add to header DiagnosticStatus
            header_status.values.append(KeyValue(name, level_str))

            # Output status message
            status_array.append(item.to_status_msg(self._prefix, stale))

        header_status.message = stat_dict[header_status.level]

        header_status.name = header_status.name 

        if all_stale:
            header_status.level = 3
            header_status.message = 'All Stale'
    
        return status_array

    ##\brief Returns messages to analyze based on topic name
    ##
    ## Checks all status names if they need analysis, updates analysis
    ## count of status names. Checks status.name against expected, startswith,
    ## contains and name lists.
    ##\param msgs dict{DiagnosticStatus, count} : Given array of messages
    def _select_msgs(self, msgs):
        to_analyze = []
        
        # If processing the remainder, return all unanalyzed messages
        if self._other:
            for msg, value in msgs.iteritems():
                if value == 0:
                    to_analyze.append(msg)

            return to_analyze

        for msg, value in msgs.iteritems():
            for expected in self._expected:
                if msg.name == expected:
                    to_analyze.append(msg)
                    msgs[msg] += 1
                    continue

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
    ##\param msgs list : List of DiagnosticStatus messages
    def _update_status_names(self, msgs):
        for msg in msgs:
            if not self._status_names.has_key(msg.name):
                self._status_names[msg.name] = DiagnosticItem(msg)
            self._status_names[msg.name].update(msg)


        
        

        
                
