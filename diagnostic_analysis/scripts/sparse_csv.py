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


# Make any csv into sparse csv


PKG = 'diagnostic_analysis'
import roslib
roslib.load_manifest(PKG)

import csv, os, sys
from optparse import OptionParser

from diagnostic_analysis.sparse import make_sparse_skip, make_sparse_length

if __name__=='__main__':
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
        print 'No CSV file given.'
        sys.exit(0)

    csv_file = args[0]

    if not csv_file.endswith('.csv'):
        print 'File %s is not a CSV file. Aborting.' % csv_file
        sys.exit(0)    
    
    if options.max:
        output_file = make_sparse_length(csv_file, 65000)
    elif options.length is None:
        output_file = make_sparse_skip(csv_file, options.skip)
    else:
        output_file = make_sparse_length(csv_file, int(options.length))

    print 'Created sparse CSV %s' % output_file
