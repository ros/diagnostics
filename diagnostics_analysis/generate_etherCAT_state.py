#!/usr/bin/env python
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

# Author: Eric Berger

PKG = 'diagnostics_analysis'
import roslib; roslib.load_manifest(PKG)
import rosrecord
import std_msgs.msg

def init():
  print "Starting to gather statistics"
  stats = {}
  stats['tracked_values'] = ['Programmed current', 'Measured current', 'Board temperature', 'Bridge temperature', 'Supply voltage', 'Drops', 'Max Consecutive Drops']
  return stats

def update(stats, topic, msg):
  if(not (topic == '/diagnostics')):
    print "discarding message on topic " + topic
    return
  tracked_values = stats['tracked_values']
  for status in msg.status:
    name = status.name
    if(name.startswith('EtherCAT Device')):
      if(not stats.has_key(name)):
        stats[name] = {} 
        for l in tracked_values:
          stats[name][l] = []
      for value in status.strings:
        if value.label in tracked_values:
          stats[name][value.label].append(float(value.value))

def output(stats):
  import pickle
  pickle.dump(stats, file('ethercat_stats.out', 'w'))
  print "Wrote file 'ethercat_stats.out'"

def process_bag(stats, update, bagfile):
  for (topic, msg, t) in rosrecord.logplayer(bagfile, raw=False):
    update(stats, topic, msg)

if __name__ == '__main__':
  import sys
  stats = init()
  for i, f in enumerate(sys.argv[1:]): 
    print "(%d) processing file: %s" %(i, f) 
    try:
      process_bag(stats, update, f)
    except:
      print "Failed to process bag " + f
  output(stats)
