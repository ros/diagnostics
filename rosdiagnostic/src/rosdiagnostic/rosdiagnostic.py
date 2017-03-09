#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Clearpath Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
import argparse
import os
import re
import rosgraph
import rospy
import socket
import sys

from datetime import datetime
from dateutil.tz import tzlocal
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


class ROSTopicException(Exception):
    """
    Base exception class of related errors
    """
    pass


class ROSTopicIOException(ROSTopicException):
    """
    Errors related to network I/O failures
    """
    pass


def _check_master():
    """
    Make sure that master is available
    :raises: :exc:`ROSTopicException` If unable to successfully communicate with master
    """
    try:
        rosgraph.Master('/rostopic').getPid()
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")


class RosDiagnostics(object):

    DIAG_MSG_LVL_STR_NOCOLOR = {
        DiagnosticStatus.OK:    ' OK  ',
        DiagnosticStatus.WARN:  'WARN ',
        DiagnosticStatus.ERROR: 'ERROR',
        DiagnosticStatus.STALE: 'STALE',
    }

    DIAG_MSG_LVL_STR_COLOR = {
        DiagnosticStatus.OK:    '\033[92m OK  \033[0m',
        DiagnosticStatus.WARN:  '\033[93mWARN \033[0m',
        DiagnosticStatus.ERROR: '\033[91mERROR\033[0m',
        DiagnosticStatus.STALE: '\033[94mSTALE\033[0m',
    }

    def __init__(self, options):
        self._lvl_strings_map = self.DIAG_MSG_LVL_STR_NOCOLOR if options.nocolor else self.DIAG_MSG_LVL_STR_COLOR
        self._options = options
        rospy.Subscriber(options.topic, DiagnosticArray, self._diag_callback, queue_size=1)

    def _get_ns(self, name):
        # Split namespace by slashes, return the base namespace.
        return '/'.join(name.split('/')[:-1])

    def _get_non_leaf_statuses(self, statuses):
        return [self._get_ns(s.name) for s in statuses]

    def _get_leaf_statuses(self, statuses):
        parent_namespaces = self._get_non_leaf_statuses(statuses)
        return [s for s in statuses if s.name not in parent_namespaces]

    def _print_begin_banner(self, msg):
        if not self._options.follow:
            dt = datetime.fromtimestamp(msg.header.stamp.to_sec(), tzlocal())
            print('=====================================================================')
            print('Diagnostics generated on: {}'.format(dt))
            print('---------------------------------------------------------------------')

    def _print_end_banner(self, msg):
        if not self._options.follow:
            print('=====================================================================')
            rospy.signal_shutdown('Run complete')

    def _diag_callback(self, msg):
        prog = re.compile(self._options.filter)
        self._print_begin_banner(msg)
        for diag in sorted(self._get_leaf_statuses(msg.status), key=lambda d: d.level, reverse=True):
            if prog.search(diag.name):
                self._print_status(diag, msg.header.stamp)
        self._print_end_banner(msg)

    def _print_status(self, diag, ts):
        if self._options.min_level <= diag.level:
            print('[ {} ] {} - {}'.format(self._lvl_strings_map[diag.level], diag.name, diag.message))
            if self._options.detail:
                dt = datetime.fromtimestamp(ts.to_sec(), tzlocal())
                print('    timestamp:   {}'.format(dt))
                print('    hardware_id: {}'.format(diag.hardware_id))
                for kv in diag.values:
                    print('    - {}: {}'.format(kv.key, kv.value))


def _rosdiagnostic_cmd_echo(argv):
    parser = argparse.ArgumentParser(
        description='ROS Diagnostic Viewer is a command-line tool for printing information about ROS Diagnostics.'
    )
    parser.add_argument('-f', '--follow', action='store_true', dest='follow',
                        help='follows the diagnostic messages continuously')

    parser.add_argument('-l', '--level', action='store', metavar='LEVEL', type=int, default=DiagnosticStatus.WARN,
                        dest='min_level',
                        help='the minimum diagnostic level to display (OK=0, WARN=1, ERROR=2, STALE=3) [default=1]')

    parser.add_argument('--topic', action='store', metavar='TOPIC', type=str, default='/diagnostics_agg', dest='topic',
                        help='topic to read the diagnostics from')

    parser.add_argument('--filter', action='store', metavar='FILTER', type=str, default='.*', dest='filter',
                        help='regular expression to be applied as a filter to the diagnostic name')

    parser.add_argument('--nocolor', action='store_true',
                        help='output should not make use of any color')

    parser.add_argument('-d', '--detail', action='store_true',
                        help='printing the full diagnostic details ')

    args = parser.parse_args(argv)

    _check_master()
    rospy.init_node('rosdiagnostic', anonymous=True)
    rosdiag = RosDiagnostics(args)
    rospy.spin()
    del rosdiag


def rosdiagnosticmain(argv=None):
    if argv is None:
        argv = sys.argv
    # filter out remapping arguments in case we are being invoked via roslaunch
    argv = rospy.myargv(argv)

    try:
        _rosdiagnostic_cmd_echo(argv[1:])
    except socket.error:
        sys.stderr.write("Network communication failed. Most likely failed to communicate with master.\n")
        sys.exit(1)
    except rosgraph.MasterException as e:
        # mainly for invalid master URI/rosgraph.masterapi
        sys.stderr.write("ERROR: %s\n" % str(e))
        sys.exit(1)
    except ROSTopicException as e:
        sys.stderr.write("ERROR: %s\n" % str(e))
        sys.exit(1)
    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
