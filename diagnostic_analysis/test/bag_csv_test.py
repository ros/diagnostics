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

# Author: Kevin Watts

PKG = 'diagnostic_analysis'

import roslib; roslib.load_manifest(PKG)
import rostest
import unittest

import rosbag
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import random
import tempfile
import time, os
import csv

from diagnostic_analysis.exporter import LogExporter
from diagnostic_analysis.sparse import *

row_count = 100

##\brief Make DiagnosticArray message for testing
def make_status_msg(count):
    array = DiagnosticArray()
    stat = DiagnosticStatus()
    stat.level = 0
    stat.message = 'OK'
    stat.name = 'Unit Test'
    stat.hardware_id = 'HW ID'
    stat.values = [
        KeyValue('Value A', str(count)),
        KeyValue('Value B', str(count)),
        KeyValue('Value C', str(count))]
    array.status = [ stat ]
    return array

##\brief Tests convert logfile to CSV and making sparse
class TestBagToCSV(unittest.TestCase):
    def setUp(self):
        # Make logfile with bogus messages
        self.bag = tempfile.NamedTemporaryFile()

        rebagger = rosbag.Bag(self.bag.name, 'w')
        for i in range(0, row_count):
            rebagger.write("/diagnostics", make_status_msg(i))
        rebagger.close()

        # Make CSV
        self.exp = LogExporter(None, self.bag.name)
        self.exp.process_log()
        self.exp.finish_logfile()
        self.filename = self.exp.get_filename('Unit Test')

        ## Make sparse CSV's
        self.skip_10 = make_sparse_skip(self.filename, 10)
        self.length_10 = make_sparse_length(self.filename, 10)

    ##\brief Tests that exported file exists and is not None
    def test_file_exists(self):
        self.assert_(self.filename is not None, "CSV file is None")
        self.assert_(os.path.isfile(self.filename), "CSV file doesn't exist")

    ##\brief Test that CSV file has correct data, number of lines
    def test_export(self):
        # Read CSV, count rows
        input_reader = csv.reader(open(self.filename, newline=''), delimiter=',')
        count = -1
        for row in input_reader:
            if count == -1:
                self.assert_(row[2].strip() == 'Message')
                self.assert_(row[3].strip() == 'Hardware ID')
                self.assert_(row[4].strip() == 'Value A')
                count += 1
                continue

            self.assert_(row[2].strip() == 'OK')
            self.assert_(row[3].strip() == 'HW ID')
            self.assert_(row[4].strip() == str(count))
            count += 1
      
        self.assert_(count == row_count, "Row count doesn't match")

    ##\brief Tests that sparse CSV made with 'skip' option has correct number of lines
    def test_sparse_skip(self):
        self.assert_(len(open(self.skip_10).read().split('\n')) <= int(row_count / 10) + 2, "Length of sparse CSV (skipped) incorrect")

    ##\brief Tests that sparse CSV made with 'length' option has correct number of lines
    def test_sparse_length(self):
        self.assert_(len(open(self.length_10).read().split('\n')) == 12, "Length of sparse CSV incorrect")

    def tearDown(self):
        self.bag.close()
        os.remove(self.skip_10)
        os.remove(self.length_10)

        self.exp.remove_files()

        
if __name__ == '__main__':
    if True: # Use rostest for accurate results
        rostest.unitrun(PKG, 'bag_csv_test', TestBagToCSV)
    else:
        # Manual test suite
        suite = unittest.TestSuite()
        suite.addTest(TestBagToCSV('test_file_exists'))
        suite.addTest(TestBagToCSV('test_export'))
        suite.addTest(TestBagToCSV('test_sparse_skip'))
        suite.addTest(TestBagToCSV('test_sparse_length'))
        
        unittest.TextTestRunner(verbosity = 2).run(suite)
