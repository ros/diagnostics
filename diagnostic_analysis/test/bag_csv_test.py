#!/usr/bin/python3.6
#
# Copyright 2018 Open Source Robotics Foundation, Inc.
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

# Author: Kevin Watts

import csv
import os
# import random
# import tempfile
# import time
import unittest

# import rosbag
from diagnostic_analysis.sparse_csv import make_sparse_length, make_sparse_skip
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
# from diagnostic_analysis.exporter import LogExporter

row_count = 100

# \brief Make DiagnosticArray message for testing


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
    array.status = [stat]
    return array

# \brief Tests convert logfile to CSV and making sparse


class TestBagToCSV(unittest.TestCase):

    def setUp(self):
        # Make logfile with bogus messages
        # self.bag = tempfile.NamedTemporaryFile()

        # rebagger = rosbag.Bag(self.bag.name, 'w')
        # for i in range(0, row_count):
        #    rebagger.write("/diagnostics", make_status_msg(i))
        # rebagger.close()

        # Make CSV
        # self.exp = LogExporter(None, self.bag.name)
        # self.exp.process_log()
        # self.exp.finish_logfile()
        # self.filename = self.exp.get_filename('Unit Test')
        self.filename = 'Unit_Test_1.csv'

        # Make sparse CSV's
        self.skip_10 = make_sparse_skip(self.filename, 10)
        self.length_10 = make_sparse_length(self.filename, 10)

    # \brief Tests that exported file exists and is not None

    def test_file_exists(self):
        self.assertTrue(self.filename is not None, 'CSV file is None')
        self.assertTrue(os.path.isfile(self.filename), 'CSV file doesnt exist')

    # \brief Test that CSV file has correct data, number of lines

    def test_export(self):
        # Read CSV, count rows
        self.filename = 'Unit_Test_1.csv'
        input_reader = csv.reader(open(self.filename, 'r'))
        count = -1
        for row in input_reader:
            if count == -1:
                self.assertTrue(row[2].strip() == 'Message')
                self.assertTrue(row[3].strip() == 'Hardware ID')
                self.assertTrue(row[4].strip() == 'Value A')
                count += 1
                continue

            self.assertTrue(row[2].strip() == 'OK')
            self.assertTrue(row[3].strip() == 'HW ID')
            self.assertTrue(row[4].strip() == str(count))
            count += 1

        self.assertTrue(count == row_count, 'Row count doesnt match')

    # \brief Tests that sparse CSV made with 'skip' option has correct number of lines

    def test_sparse_skip(self):
        self.assertTrue(len(open(self.skip_10).read().split('\n')) <= int(row_count / 10) + 2,
                        'Length of sparse CSV (skipped) incorrect')

    # \brief Tests that sparse CSV made with 'length' option has correct number of lines

    def test_sparse_length(self):
        self.assertTrue(len(open(self.length_10).read().split('\n')) == 12,
                        'Length of sparse CSV incorrect')

    def tearDown(self):
        # self.bag.close()
        os.remove(self.skip_10)
        os.remove(self.length_10)
        # self.exp.remove_files()


if __name__ == '__main__':
    unittest.main()
