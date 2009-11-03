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

##\author Kevin Watts
##\brief Make CSV files smaller for use in spreadsheet software

PKG = 'diagnostic_analysis'
import roslib
roslib.load_manifest(PKG)

import csv, os, sys

##\brief Makes sparse CSV by skipping every nth value
##\param csv_file str : CSV filename
##\param skip int : Write every nth row to sparse CSV
##\return Path of output file
def make_sparse_skip(csv_file, skip):
    output_file = csv_file[:-4] + '_sparse.csv'

    input_reader = csv.reader(open(csv_file, 'rb'))

    f = open(output_file, 'wb')
    output_writer = csv.writer(f)

    skip_count = skip
    for row in input_reader:
        if skip_count == skip:
            output_writer.writerow(row)
            skip_count = 0
            
        skip_count = skip_count + 1

    return output_file

##\brief Makes sparse CSV with the given number of rows
##\param csv_file str : CSV filename
##\param length int : Desired number of rows in CSV
##\return Path of output file
def make_sparse_length(csv_file, length):
    output_file = csv_file[:-4] + '_sprs_len.csv'

    input_reader = csv.reader(open(csv_file, 'rb'))

    f = open(output_file, 'wb')
    output_writer = csv.writer(f)

    # Calculate skip count for file
    orig_len = len(open(csv_file, 'r').read().split('\n'))
    skip = max(int(orig_len / length), 1)

    skip_count = skip
    for row in input_reader:
        if skip_count >= skip:
            output_writer.writerow(row)
            skip_count = 0
            
        skip_count = skip_count + 1

    return output_file
