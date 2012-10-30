#!/usr/bin/python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

##\author Kevin Watts
##\brief LogExporter class does diagnostics logfile conversion to CSV

import roslib
import rosbag
import diagnostic_msgs.msg
import time, sys, os
import operator, tempfile, subprocess

##\brief Converts and processes diagnostics logs to CSV format
##
## Used by scripts/export_csv.py to convert diagnostics log files to CSV format
class LogExporter:
    ##\param output_dir str : Complete path of output dir. If None, uses temp dir
    ##\param logfile str : path of logfile
    def __init__(self, output_dir, logfile):
        self._temp = False
        self._stats = {}
        self.logfile = logfile

        self.output_dir = output_dir
        if self.output_dir is None:
            self.output_dir = tempfile.mkdtemp()
            self._temp = True
        
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)

    ##\brief Removes all output files. Removes directory if temp
    def remove_files(self):
        for name in self._stats:
            file = self._stats[name]['file_name']
            os.remove(file)
        if self._temp:
            os.rmdir(self.output_dir)
        
    ##\brief Return filename of output
    ##\param name str : DiagnosticStatus name ex: 'Mechanism Control'
    def get_filename(self, name):
        if not self._stats.has_key(name):
            return None # self.output_dir + '/%s.csv' % name.replace(' ', '_')
        return self._stats[name]['file_name']

    ##\brief Use rosrecord to play back bagfile
    def process_log(self):
        bag = rosbag.Bag(self.logfile)
        for (topic, msg, t) in bag.read_messages():
            self._update(topic, msg)

    ##\brief Creates and updates data files with new messages
    def _update(self, topic, msg):
        if (not (topic == '/diagnostics')):
            print "Discarding message on topic: %s" % topic
            return
        
        t = time.localtime(float(str(msg.header.stamp)) / 1000000000.0)
        
        for status in msg.status:
            name = status.name

            if (not self._stats.has_key(name)):
                self._stats[name] = {}
                
                fields = {}
                for index, s in enumerate(status.values):
                    fields[s.key] = index
                    
                self._stats[name]['fields'] = fields
                
                self._stats[name]['level'] = status.level
                self._stats[name]['message'] = status.message
                self._stats[name]['hardware_id'] = status.hardware_id

                # Use named temp file, will cat this to header on close
                datafile, tmp_name = tempfile.mkstemp()
                self._stats[name]['data_file'] = os.fdopen(datafile, 'w')
                self._stats[name]['data_name'] = tmp_name
                

            # Check to see if fields have changed. Add new fields to map
            if (not [s.key for s in status.values] == self._stats[name]['fields'].keys()):
                for s in status.values:
                    if not self._stats[name]['fields'].has_key(s.key):
                        self._stats[name]['fields'][s.key] = len(self._stats[name]['fields'])

            # Add values in correct place for header index
            # Key/Value pairs can move around, this makes sure values are 
            # added to correct keys
            vals = []
            for key, val in self._stats[name]['fields'].iteritems():
                vals.append('')
            for s in status.values:
                vals[self._stats[name]['fields'][s.key]] = s.value.replace('\n','  ').replace(',',' ')
        
            msg = status.message.replace(',',' ').strip()
            hw_id = status.hardware_id.replace(',', ' ')
        
            self._stats[name]['data_file'].write(','.join([time.strftime("%Y/%m/%d %H:%M:%S", t)] + 
                                            [str(status.level), msg, hw_id] + vals) + '\n')

    ##\brief Close logfile, append data to header
    def finish_logfile(self):
        for name in self._stats:
            # Sort fields by correct index, add to header
            field_dict = sorted(self._stats[name]['fields'].iteritems(), key=operator.itemgetter(1))
            fields = map(operator.itemgetter(0), field_dict)
            
            header_line = ','.join(['Timestamp'] + ['Level', 'Message', 'Hardware ID'] + 
                                    [f.replace(',','').replace('\n', ' ') for f in fields]) + '\n'
            
            file_name = os.path.join(self.output_dir, name.replace(' ', '_').replace('(', '').replace(')', '').replace('/', '__').replace('.', '').replace('#', '') + '.csv')
            
            output_file = file(file_name, 'w')
            output_file.write(header_line)
            output_file.close()


            self._stats[name]['data_file'].close() # Close data
            
            # Append the tmp data file to the header
            subprocess.call("cat %s >> %s" % (self._stats[name]['data_name'], file_name), shell=True)
            # Remove tmp file
            os.remove(self._stats[name]['data_name'])
            # Store filename
            self._stats[name]['file_name'] = file_name
    
