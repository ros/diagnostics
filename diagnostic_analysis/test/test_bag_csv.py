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

PKG = "diagnostic_analysis"

import csv
import os
import tempfile
import time
import unittest

import rosbag2_py
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.serialization import serialize_message

from diagnostic_analysis.exporter import LogExporter
from diagnostic_analysis.sparse import *

row_count = 100


##\brief Make DiagnosticArray message for testing
def make_status_msg(count):
    array = DiagnosticArray()

    array.header.stamp.sec = int(time.time())
    array.header.stamp.nanosec = int((time.time() - int(time.time())) * 1e9)

    stat = DiagnosticStatus()

    stat.level = DiagnosticStatus.OK

    stat.message = "OK"
    stat.name = "Unit Test"
    stat.hardware_id = "HW ID"
    stat.values = [
        KeyValue(key="Value A", value=str(count)),
        KeyValue(key="Value B", value=str(count)),
        KeyValue(key="Value C", value=str(count)),
    ]
    array.status = [stat]
    return array


##\brief Tests convert logfile to CSV and making sparse
class TestBagToCSV(unittest.TestCase):
    def setUp(self):
        # Make logfile with bogus messages
        tmp_dir = tempfile.mkdtemp()
        bag_path = os.path.join(tmp_dir, "my_bag")
        self.bag_path = bag_path

        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=bag_path),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )

        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=0,
                name="/diagnostics",
                type="diagnostic_msgs/msg/DiagnosticArray",
                serialization_format="cdr",
            )
        )

        for i in range(row_count):
            msg = make_status_msg(i)
            serialized_msg = serialize_message(msg)
            timestamp_ns = time.time_ns()
            writer.write("/diagnostics", serialized_msg, timestamp_ns)

        del writer

        # Make CSV
        self.exp = LogExporter(None, bag_path)
        self.exp.process_log()
        self.exp.finish_logfile()
        self.filename = self.exp.get_filename("Unit Test")

        ## Make sparse CSV's
        self.skip_10 = make_sparse_skip(self.filename, 10)
        self.length_10 = make_sparse_length(self.filename, 10)

    ##\brief Tests that exported file exists and is not None
    def test_file_exists(self):
        assert self.filename is not None, "CSV file is None"
        assert os.path.isfile(self.filename), "CSV file doesn't exist"

    ##\brief Test that CSV file has correct data, number of lines
    def test_export(self):
        # Read CSV, count rows
        with open(self.filename,'r') as f:
            input_reader = csv.reader(f)
            count = -1
            for row in input_reader:
                if count == -1:
                    assert row[2].strip() == "Message"
                    assert row[3].strip() == "Hardware ID"
                    assert row[4].strip() == "Value A"
                    count += 1
                    continue

                assert row[2].strip() == "OK"
                assert row[3].strip() == "HW ID"
                assert row[4].strip() == str(count)
                count += 1

            assert count == row_count, "Row count doesn't match"

    ##\brief Tests that sparse CSV made with 'skip' option has correct number of lines
    def test_sparse_skip(self):
        with open(self.skip_10) as f:
            assert len(f.read().split("\n")) <= int(row_count / 10) + 2, (
                "Length of sparse CSV (skipped) incorrect"
            )

    ##\brief Tests that sparse CSV made with 'length' option has correct number of lines
    def test_sparse_length(self):
        with open(self.length_10) as f:
            assert len(f.read().split("\n")) == 12, "Length of sparse CSV incorrect"

    def tearDown(self):
        os.remove(self.skip_10)
        os.remove(self.length_10)

        self.exp.remove_files()


if __name__ == "__main__":
    suite = unittest.TestSuite()
    suite.addTest(TestBagToCSV("test_file_exists"))
    suite.addTest(TestBagToCSV("test_export"))
    suite.addTest(TestBagToCSV("test_sparse_skip"))
    suite.addTest(TestBagToCSV("test_sparse_length"))

    unittest.TextTestRunner(verbosity=2).run(suite)
