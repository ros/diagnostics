#!/usr/bin/env python
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

## Aggregates diagnostics from robot and republishes to /diagnostics_agg

PKG = 'diagnostic_aggregator'
import roslib; roslib.load_manifest(PKG)

import rospy

from xml.dom import minidom

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import traceback

import sys

import threading
from threading import Timer

##\brief Creates analyzer class from XML snippet
##@param xml_node mindom node : XML node to choose, initialize analyzer
##@param global_prefix str : Prefix of all analyzers for this aggregator
def create_analyzer(xml_node, global_prefix):
    type   = xml_node.attributes['type'].value
    pkg    = xml_node.attributes['pkg'].value
    file   = xml_node.attributes['file'].value
    local_prefix = xml_node.attributes['prefix'].value
    
    prefix = '/'.join([global_prefix, local_prefix])

    if pkg != PKG:
        roslib.load_manifest(pkg)
        
    import_str = '%s.%s' % (pkg, file)
    __import__(import_str)

    try:
        pypkg = sys.modules[import_str]
        my_analyzer = getattr(pypkg, type) # Type of analyzer
    except KeyError:
        rospy.logfatal("Cannot load package %s, module %s" % (pkg, file))
        sys.exit(-1)

    try:
        # Create dynamically
        analyzer = my_analyzer(xml_node, prefix)
    except:
        rospy.logfatal('Failed to create analyzer. %s' % traceback.format_exc())
        sys.exit(-1)

    # Check that it has attributes analyze, topic
        
    return analyzer

##\brief Aggregates /diagnostics topic and republishes on /diagnostics_agg
## 
## DiagnosticAggregator ia an aggregator that creates "analyzers" to 
## analyze, store and track DiagnosticStatus messages. It publishes the 
## aggregated data on /diagnostics_agg as a DiagnosticArray. It prepends the 
## given prefix to all aggregated DiagnosticStatus message names.
## 
## The XML snippet to initialize the aggregator contains the necessary 
## data to dynamically load the analyzers.
##\verbatim
## <aggregator>
##  <analyzer type="AnalyzerClassName" pkg="analyzer_package"
##            file="analyzer_file" prefix="Prefix" >
##   <!-- Anything else to initialize the analyzer -->
##  </analyzer>
## <!-- Other analyzers -->
## </aggregator>
##\endverbatim
## The analyzer will prepend the prefix "/aggregator_prefix/Prefix/" to 
## all topics it analyzes. 
class DiagnosticAggregator:
    ##@param prefix str : Prepended to all processed DiagnosticStatus names
    ##@param xml_doc minidom node : XML node for analyzer creation
    def __init__(self, prefix, xml_doc):
        self._analyzers = []
        self._msgs = {}

        self._mutex = threading.Lock()

        if not prefix.startswith('/'):
            prefix = '/' + prefix
        self._prefix = prefix
        
        analyzers = xml_doc.getElementsByTagName('analyzer')
        for analyzer in analyzers:
            self._analyzers.append(create_analyzer(analyzer, self._prefix))

        rospy.Subscriber('/diagnostics', DiagnosticArray, self.diag_callback)

        self._agg_pub = rospy.Publisher('/diagnostics_agg', DiagnosticArray)

    ##\brief Callback for /diagnostics subscription
    def diag_callback(self, array):
        self._mutex.acquire()
        
        for msg in array.status:
            self._msgs[msg] = 0
        self._mutex.release()

    ##\brief Calls analyzers, publishes /diagnostics_agg 
    def publish_data(self):
        self._mutex.acquire()

        # Global status
        status = DiagnosticStatus()
        status.name = self._prefix
        status.level = 0
        status.message = 'OK'
        status.values = []

        array = DiagnosticArray()
        
        array.status = [ status ]

        for analyzer in self._analyzers:
            array.status.extend(analyzer.analyze(self._msgs))
            
        array.status.extend(self._process_remainder())
        
        self._agg_pub.publish(array)
        self._msgs = {}

        self._mutex.release()

    ##\brief Process remaining messages with 'Other' prefix
    def _process_remainder(self):
        remainder_header = DiagnosticStatus()
        remainder_header.name = '%s/%s' % (self._prefix, 'Other')
        remainder_header.level = 0
        remainder_header.message = 'OK'
        remainder_header.values = []

        remainder = [ remainder_header ]
        for msg, value in self._msgs.iteritems():
            if value > 0:
                continue # Has been processed

            msg.name = '%s/%s/%s' % (self._prefix, 'Other', msg.name)
            remainder.append(msg)

        return remainder
        
 
