#!/usr/bin/python
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
import diagnostic_msgs.msg
import time, sys
import traceback
import os

def init():
  print "Starting to gather statistics"
  stats = {}

  return stats



def update(stats, topic, msg):
  global output_dir

  if(not (topic == '/diagnostics')):
    print "discarding message on topic " + topic
    return
  t = time.localtime(float(str(msg.header.stamp)) / 1000000000.0)
   
  for status in msg.status:
    name = status.name

    if(not stats.has_key(name)):
      stats[name] = {}
      stats[name]['string_fields'] = [s.key for s in status.strings]
      stats[name]['float_fields'] = [s.key for s in status.values]
      stats[name]['level'] = status.level
      stats[name]['message'] = status.message
      
      file_name = name.replace(' ', '_').replace('(', '').replace(')', '').replace('/', '__').replace('.', '').replace('#', '')
      stats[name]['file'] = file(output_dir + '/' + file_name + '.csv', 'w')

      fields = stats[name]['string_fields'] + stats[name]['float_fields'];
      stats[name]['file'].write(', '.join(['timestamp'] + ['Level', 'Message'] + 
                                          [f.replace(',','') for f in fields]) + '\n')
    
    # Need stuff for different string fields
    # Store as dictionary, then convert to CSV?
    if (not [s.key for s in status.strings] == stats[name]['string_fields']):
      #print "ERROR, mismatched field names in component %s. Label: %s" %(name, s.key)
      #print [s.key for s in status.strings]
      #print str(stats[name]['string_fields'])
      #return stats
      continue
    if (not [s.key for s in status.values] == stats[name]['float_fields']):
      #print "ERROR, mismatched field names in component %s. Label: %s" %(name, s.key)
      #return stats
      continue

    # Should make time machine readable better
    msg = status.message.replace(',',' ')

    stats[name]['file'].write(', '.join([time.asctime(t)] + [str(status.level), msg] + 
                                        [s.value.replace('\n', ' ').replace(',','') for s in status.strings] + 
                                        [str(s.value) for s in status.values]) + '\n')


def output(stats):
  for name in stats:
    stats[name]['file'].close()


def process_bag(stats, update, bagfile):
  for (topic, msg, t) in rosrecord.logplayer(bagfile, raw=False):
    update(stats, topic, msg)

if __name__ == '__main__':
  stats = init()
  try:
    for i, f in enumerate(sys.argv[1:]): 
      global output_dir
      filepath = 'output/%s_csv' % os.path.basename(f)[0:os.path.basename(f).find('.')]
      
      output_dir = os.path.join(roslib.packages.get_pkg_dir(PKG),  filepath)
      print "(%d) processing file: %s. Output dir: %s" %(i, f, filepath) 
      if not os.path.isdir(output_dir):
        os.mkdir(output_dir)

      process_bag(stats, update, f)
      output(stats)
  except:
    traceback.print_exc()
