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

# Author: Eric Berger, Kevin Watts

##\brief Converts diagnostics log files into CSV's for analysis

PKG = 'diagnostics_analysis'
import roslib; roslib.load_manifest(PKG)
import rosrecord #, rostime
import diagnostic_msgs.msg
import time, sys, os
import operator, tempfile, subprocess

from optparse import OptionParser

stats = {}

##\brief Adds message to output file
def update(topic, msg):
    global output_dir

    if(not (topic == '/diagnostics')):
        print "Discarding message on topic: %s" % topic
        return

    t = time.localtime(float(str(msg.header.stamp)) / 1000000000.0)
   
    for status in msg.status:
        name = status.name

        if(not stats.has_key(name)):
            stats[name] = {}
            #stats[name]['fields'] = [s.key for s in status.values]

            fields = {}
            index = 0
            for s in status.values:
                fields[s.key] = index
                index += 1
            stats[name]['fields'] = fields

            stats[name]['level'] = status.level
            stats[name]['message'] = status.message
            # Use named temp file, will cat this to header on close
            stats[name]['file'] = tempfile.NamedTemporaryFile()

            #fields = stats[name]['fields'];

        # Check to see if fields have changed. Add new fields to map
        if (not [s.key for s in status.values] == stats[name]['fields'].keys()):
            for s in status.values:
                if not stats[name]['fields'].has_key(s.key):
                    stats[name]['fields'][s.key] = len(stats[name]['fields'])
                    
        # Add values in correct place for header index
        # Key/Value pairs can move around, this makes sure values are added to correct keys
        vals = []
        for key, val in stats[name]['fields'].iteritems():
            vals.append('')
        for s in status.values:
            vals[stats[name]['fields'][s.key]] = s.value.replace('\n','  ').replace(',',' ')
        
        msg = status.message.replace(',',' ')
        
        stats[name]['file'].write(', '.join([time.strftime("%Y/%m/%d %H:%M:%S", t)] + 
                                            [str(status.level), msg] + vals) + '\n')

##\brief Saves output file with complete header row
def finish_file():
    for name in stats:
        # Sort fields by correct index, add to header
        field_dict = sorted(stats[name]['fields'].iteritems(), key=operator.itemgetter(1))
        fields = map(operator.itemgetter(0), field_dict)

        header_line = ', '.join(['Timestamp'] + ['Level', 'Message'] + [f.replace(',','').replace('\n', ' ') for f in fields]) + '\n'

        file_name = os.path.join(output_dir, name.replace(' ', '_').replace('(', '').replace(')', '').replace('/', '__').replace('.', '').replace('#', '') + '.csv')
        
        output_file = file(file_name, 'w')
        output_file.write(header_line)
        output_file.close()

        # Cat the temp data to the header file and close it
        subprocess.call("cat %s >> %s" % (stats[name]['file'].name, file_name), shell=True)
        stats[name]['file'].close() # Destroy temp file

##\brief Plays bagfile and updates logs with new messages
def process_bag(bagfile):
    for (topic, msg, t) in rosrecord.logplayer(bagfile, raw=False):
        update(topic, msg)


if __name__ == '__main__':
    # Allow user to set output directory
    parser = OptionParser()
    parser.add_option("-d", "--directory", dest="directory",
                      help="Write output to DIR. Default: %s" % PKG, metavar="DIR",
                      default=roslib.packages.get_pkg_dir(PKG), action="store")
    options, args = parser.parse_args()

    try:
        for i, f in enumerate(args):
            global output_dir
            filepath = 'output/%s_csv' % os.path.basename(f)[0:os.path.basename(f).find('.')]
            
            output_dir = os.path.join(options.directory,  filepath)
            print "(%d) processing file: %s. Output dir: %s/output" %(i, f, options.directory)
            if not os.path.isdir(output_dir):
                os.makedirs(output_dir)
                
            process_bag(f)
            finish_file()
    except:
        import traceback
        print "Caught exceptiong processing log file"
        traceback.print_exc()
