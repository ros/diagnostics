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

# Author: Kevin Watts

import pylab
import pickle
import sys
import roslib


def plot_key(label, stats):
    boards = [b for b in stats.keys() if b.count('motor') > 0]
    for board in boards:
        pylab.plot(stats[board][label], label=board)

    pylab.title(label + ' Timeseries')
    pylab.xlabel('Message Number')
    pylab.ylabel(label)
    pylab.legend()

    pylab.show()

def main():
    if len(sys.argv) < 2:
        print 'plot_timeseries_data.py <stats.out file>'
        print 'Plots data from given file for chosen key'
        sys.exit(1)

    stats = pickle.load(file(sys.argv[1]))

    keys = stats['tracked_values']
    key_str_list = ''
    key_index = {}
    for index, key in enumerate(keys):
        key_index[index] = key
        key_str_list += "%d %s\n" % (index, key)
    while(1):
        print "Available values:\n" + key_str_list
        try:
            index = input("What value would you like to plot (give index)? ")
            key = key_index[index]
            if key in keys:
                plot_key(key, stats)
            else:
                print "Error, key not found"
        except:
            traceback.print_exc()
            return


if __name__ == '__main__':
    main()
