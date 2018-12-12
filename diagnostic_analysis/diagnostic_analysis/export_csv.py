#!/usr/bin/python3.6
#
# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

##\author Eric Berger, Kevin Watts

##\brief Converts diagnostics log files into CSV's for analysis

PKG = 'diagnostic_analysis'
import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from rclpy.qos import QoSReliabilityPolicy

import diagnostic_msgs.msg
import time, sys, os
import operator, tempfile, subprocess

from optparse import OptionParser

#from diagnostic_analysis.exporter import LogExporter
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




def main(argv=sys.argv[1:]):
    parser = OptionParser()
    parser.add_option("-d", "--directory", dest="directory", help="Write output to DIR/output. Default: %s" % PKG, metavar="DIR", action="store" )

    options, args = parser.parse_args()

    exporters = []

    #print 'Output directory: %s/output' % options.directory
    print("Output directory: %s/output",options.directory)

    try:
        for i, f in enumerate(args):
            filepath = 'output/%s_csv' % os.path.basename(f)[0:os.path.basename(f).find('.')]
            
            output_dir = os.path.join(options.directory,  filepath)
            print("Processing file %s. File %d of %d.",(os.path.basename(f), i + 1, len(args)))
            
            exp = LogExporter(output_dir, f)
            exp.process_log()
            exp.finish_logfile()
            exporters.append(exp)

        print("Finished processing files.")
    except:
        import traceback
        print("Caught exception processing log file")
        traceback.print_exc()
if __name__ == '__main__':
    main()
