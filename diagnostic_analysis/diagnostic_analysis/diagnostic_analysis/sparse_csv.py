#!/usr/bin/python3
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
# Author: Kevin Watts

# Make any csv into sparse csv

PKG = 'diagnostic_analysis'

import csv, os, sys
from optparse import OptionParser

#from diagnostic_analysis.sparse import make_sparse_skip, make_sparse_length
import csv, os, sys

##\brief Makes sparse CSV by skipping every nth value
##\param csv_file str : CSV filename
##\param skip int : Write every nth row to sparse CSV
##\return Path of output file
def make_sparse_skip(csv_file, skip):
    output_file = csv_file[:-4] + '_sparse.csv'

    input_reader = csv.reader(open(csv_file, 'r', encoding = "utf8"))

    f = open(output_file, 'w')
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

    input_reader = csv.reader(open(csv_file, 'r',encoding = "utf8"))

    f = open(output_file, 'w')
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

def main():
    # Allow user to set output directory
    parser = OptionParser()
    parser.add_option("-l", "--length", dest="length",
                      help="Set length of output CSV", metavar="LEN",
                      default=None, action="store")
    parser.add_option("-s", "--skip", dest="skip",
                      help="Skip every nth row. If length set, will ignore this value.", 
                      metavar="SKIP", default=10, action="store")
    parser.add_option("-m", "--max", dest="max", 
                      help="Make largest possible file for Open Office (65k lines). If selected, other options ignored.",
                      metavar="MAX", default=False, action="store_true")

    options, args = parser.parse_args()

    # Get CSV file
    if len(args) < 1:
        print("No CSV file given.")
        sys.exit(0)

    csv_file = args[0]

    if not csv_file.endswith('.csv'):
        print("File %s is not a CSV file. Aborting.", csv_file)
        sys.exit(0)    
    
    if options.max:
        output_file = make_sparse_length(csv_file, 65000)
    elif options.length is None:
        output_file = make_sparse_skip(csv_file, options.skip)
    else:
        output_file = make_sparse_length(csv_file, int(options.length))

    print("Created sparse CSV %s",output_file)

if __name__=='__main__':
    main()
