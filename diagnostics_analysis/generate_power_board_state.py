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
import sys

def init():
  print "Starting to gather statistics"
  stats = {}
  return stats

def update(stats, topic, msg):
  if(not (topic == '/diagnostics')):
    print "discarding message on topic " + topic
    return
  for status in msg.status:
    name = status.name
    if(name.startswith('Power board')):
      for value in status.strings:
        if not value.label in stats:
          stats[value.label] = []
        stats[value.label].append(value.value)
      for value in status.values:
        if not value.label in stats:
          stats[value.label] = []
        stats[value.label].append(value.value)

def output(stats):
  import pickle
  pickle.dump(stats, file('power_board_stats.out', 'w'))
  print "Wrote file 'power_board_stats.out'"

def process_bag(stats, update, bagfile):
  for (topic, msg, t) in rosrecord.logplayer(bagfile, raw=False):
    update(stats, topic, msg)

if __name__ == '__main__':
  if len(sys.argv) < 2:
    print 'generate_power_board_state.py <pr2_log_file_name.bag>'
    print 'Returns parsed file as \'power_board_stats.out\' with all '
    print 'data that starts with \'Power board\' in the bag file.'
    sys.exit(1)
  
  stats = init()
  for i, f in enumerate(sys.argv[1:]): 
    print "(%d) processing file: %s" %(i, f) 
    try:
      process_bag(stats, update, f)
    except:
      print "Failed to process bag " + f
  output(stats)
