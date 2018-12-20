#! /usr/bin/env python3
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
# -*- coding: utf-8 -*-

import socket
import subprocess
import sys
import threading
from time import sleep
import traceback

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import rclpy
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.time import Time


low_hd_level = 5
critical_hd_level = 1

hd_temp_warn = 55  # 3580, setting to 55C to after checking manual
hd_temp_error = 70  # Above this temperature, hard drives will have serious problems

stat_dict = {0: 'OK', 1: 'Warning', 2: 'Error'}
temp_dict = {b'\x00': 'OK', b'\x01': 'Hot', b'\x02': 'Critical Hot'}
usage_dict = {0: 'OK', 1: 'Low Disk Space', 2: 'Very Low Disk Space'}

REMOVABLE = ['/dev/sg1', '/dev/sdb']  # Store removable drives so we can ignore if removed


#  Connects to hddtemp daemon to get temp, HD make.
def get_hddtemp_data(hostname='localhost', port=7634):
    try:
        hd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        hd_sock.connect((hostname, port))
        sock_data = ''
        while True:
            newdat = hd_sock.recv(1024)
            if len(newdat) == 0:
                break
            sock_data = sock_data + newdat
        hd_sock.close()

        sock_vals = sock_data.split('|')

        #  Format of output looks like ' | DRIVE | MAKE | TEMP | '
        idx = 0

        drives = []
        makes = []
        temps = []
        while idx + 5 < len(sock_vals):
            this_drive = sock_vals[idx + 1]
            this_make = sock_vals[idx + 2]
            this_temp = sock_vals[idx + 3]

            #  Sometimes we get duplicate makes if hard drives are mounted
            #  to two different points
            if this_make in makes:
                idx += 5
                continue

            drives.append(this_drive)
            makes.append(this_make)
            temps.append(this_temp)

            idx += 5

        return True, drives, makes, temps
    except Exception:
        return False, ['Exception'], [traceback.format_exc()], ['0']


def update_status_stale(stat, last_update_time):
    clock = Clock(clock_type=ClockType.STEADY_TIME)
    now = clock.now()
    time_since_update = now - last_update_time
    stale_status = 'OK'
    duration1 = Duration(nanoseconds=20e9)
    duration2 = Duration(nanoseconds=35e9)
    if time_since_update > duration1 and time_since_update <= duration2:
        stale_status = 'Lagging'
        if stat.level == DiagnosticStatus.OK:
            stat.message = stale_status
        elif stat.message.find(stale_status) < 0:
            stat.message = ', '.join([stat.message, stale_status])
        stat.level = max(stat.level, DiagnosticStatus.WARN)
    if time_since_update > duration2:
        stale_status = 'Stale'
        if stat.level == DiagnosticStatus.OK:
            stat.message = stale_status
        elif stat.message.find(stale_status) < 0:
            stat.message = ', '.join([stat.message, stale_status])
        stat.level = max(stat.level, DiagnosticStatus.ERROR)

    stat.values.pop(0)
    stat.values.pop(0)
    stat.values.insert(0, KeyValue(key='Update Status', value=stale_status))
    stat.values.insert(1, KeyValue(key='Time Since Update', value=str(time_since_update)))


class hd_monitor():

    def __init__(self, hostname, node, diag_hostname, home_dir=''):
        self._mutex = threading.Lock()

        self._hostname = hostname
        self._no_temp_warn = False
        self._home_dir = home_dir
        self.node = node
        self._diag_pub = self.node.create_publisher(DiagnosticArray, '/diagnostics')
        self._last_temp_time = Time(nanoseconds=0, clock_type=ClockType.STEADY_TIME)
        self._last_usage_time = Time(nanoseconds=0, clock_type=ClockType.STEADY_TIME)
        self._last_publish_time = Time(nanoseconds=0, clock_type=ClockType.STEADY_TIME)

        self._temp_timer = None
        self._usage_timer = None

        self._temp_stat = DiagnosticStatus()
        self._temp_stat.name = '%s HD Temperature' % diag_hostname
        self._temp_stat.level = DiagnosticStatus.ERROR
        self._temp_stat.hardware_id = hostname
        self._temp_stat.message = 'No Data'
        self._temp_stat.values = [KeyValue(key='Update Status', value='No Data'),
                                  KeyValue(key='Time Since Last Update', value='N/A')]

        if self._home_dir != '':
            self._usage_stat = DiagnosticStatus()
            self._usage_stat.level = DiagnosticStatus.ERROR
            self._usage_stat.hardware_id = hostname
            self._usage_stat.name = '%s HD Usage' % diag_hostname
            self._usage_stat.values = [KeyValue(key='Update Status', value='No Data'),
                                       KeyValue(key='Time Since Last Update', value='N/A')]
            self.check_disk_usage()

        self.check_temps()

    #  Must have the lock to cancel everything
    def cancel_timers(self):
        if self._temp_timer:
            self._temp_timer.cancel()
            self._temp_timer = None

        if self._usage_timer:
            self._usage_timer.cancel()
            self._usage_timer = None

    def check_temps(self):
        if not rclpy.ok():
            with self._mutex:
                self.cancel_timers()
            return

        diag_strs = [KeyValue(key='Update Status', value='OK'),
                     KeyValue(key='Time Since Last Update', value='0')]
        diag_level = DiagnosticStatus.OK
        #  diag_message = 'OK'

        temp_ok, drives, makes, temps = get_hddtemp_data()

        for index in range(0, len(drives)):
            temp = temps[index]

            #  if not unicode(temp).isnumeric() and drives[index] not in REMOVABLE:
            if not str(temp).isnumeric() and drives[index] not in REMOVABLE:
                temp_level = DiagnosticStatus.ERROR
                temp_ok = False
            #  elif not unicode(temp).isnumeric() and drives[index] in REMOVABLE:
            elif not str(temp).isnumeric() and drives[index] in REMOVABLE:
                temp_level = DiagnosticStatus.OK
                temp = 'Removed'
            else:
                temp_level = DiagnosticStatus.OK
                if float(temp) > hd_temp_warn:
                    temp_level = DiagnosticStatus.WARN
                if float(temp) > hd_temp_error:
                    temp_level = DiagnosticStatus.ERROR

            diag_level = max(diag_level, temp_level)
            diag_strs.append(KeyValue(
                key='Disk %d Temp Status' % index, value=temp_dict[temp_level]))
            diag_strs.append(KeyValue(key='Disk %d Mount Pt.' % index, value=drives[index]))
            diag_strs.append(KeyValue(key='Disk %d Device ID' % index, value=makes[index]))
            diag_strs.append(KeyValue(key='Disk %d Temp' % index, value=temp))

        if not temp_ok:
            diag_level = DiagnosticStatus.ERROR

        with self._mutex:
            clock = Clock(clock_type=ClockType.STEADY_TIME)
            self._last_temp_time = clock.now()
            self._temp_stat.values = diag_strs
            self._temp_stat.level = diag_level

            #  Give No Data message if we have no reading
            self._temp_stat.message = temp_dict[diag_level]
            if not temp_ok:
                self._temp_stat.message = 'Error'

            if self._no_temp_warn and temp_ok:
                self._temp_stat.level = DiagnosticStatus.OK

            if rclpy.ok():
                self._temp_timer = threading.Timer(10.0, self.check_temps)
                self._temp_timer.start()
            else:
                self.cancel_timers()

    def check_disk_usage(self):
        if not rclpy.ok():
            with self._mutex:
                self.cancel_timers()
            return

        diag_vals = [KeyValue(key='Update Status', value='OK'),
                     KeyValue(key='Time Since Last Update', value='0')]
        diag_level = DiagnosticStatus.OK
        diag_message = 'OK'

        try:
            p = subprocess.Popen(['df', '-P', '--block-size=1G', self._home_dir],
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if (retcode == 0):

                diag_vals.append(KeyValue(key='Disk Space Reading', value='OK'))
                row_count = 0
                for row in stdout.split('\n'):
                    if len(row.split()) < 2:
                        continue
                    if not str(row.split()[1]).isnumeric() or float(row.split()[1]) < 10:
                        continue

                    row_count += 1
                    g_available = row.split()[-3]
                    name = row.split()[0]
                    size = row.split()[1]
                    mount_pt = row.split()[-1]

                    if (float(g_available) > low_hd_level):
                        level = DiagnosticStatus.OK
                    elif (float(g_available) > critical_hd_level):
                        level = DiagnosticStatus.WARN
                    else:
                        level = DiagnosticStatus.ERROR

                    diag_vals.append(KeyValue(
                        key='Disk %d Name' % row_count, value=name))
                    diag_vals.append(KeyValue(
                        key='Disk %d Available' % row_count, value=g_available))
                    diag_vals.append(KeyValue(
                        key='Disk %d Size' % row_count, value=size))
                    diag_vals.append(KeyValue(
                        key='Disk %d Status' % row_count, value=stat_dict[level]))
                    diag_vals.append(KeyValue(
                        key='Disk %d Mount Point' % row_count, value=mount_pt))

                    diag_level = max(diag_level, level)
                    diag_message = usage_dict[diag_level]

            else:
                diag_vals.append(KeyValue(key='Disk Space Reading', value='Failed'))
                diag_level = DiagnosticStatus.ERROR
                diag_message = stat_dict[diag_level]

        except Exception:
            diag_vals.append(KeyValue(key='Disk Space Reading', value='Exception'))
            diag_vals.append(KeyValue(key='Disk Space Ex', value=traceback.format_exc()))
            diag_level = DiagnosticStatus.ERROR
            diag_message = stat_dict[diag_level]

        with self._mutex:
            clock = Clock(clock_type=ClockType.STEADY_TIME)
            self._last_temp_time = clock.now()
            self._usage_stat.values = diag_vals
            self._usage_stat.message = diag_message
            self._usage_stat.level = diag_level

            if rclpy.ok():
                self._usage_timer = threading.Timer(5.0, self.check_disk_usage)
                self._usage_timer.start()
            else:
                self.cancel_timers()

    def publish_stats(self):
        with self._mutex:
            update_status_stale(self._temp_stat, self._last_temp_time)

            msg = DiagnosticArray()
            clock = Clock(clock_type=ClockType.STEADY_TIME)
            now = clock.now()
            msg.header.stamp = now.to_msg()
            msg.status.append(self._temp_stat)
            if self._home_dir != '':
                update_status_stale(self._usage_stat, self._last_usage_time)
                msg.status.append(self._usage_stat)

            clock = Clock(clock_type=ClockType.STEADY_TIME)
            duration3 = Duration(nanoseconds=5e8)
            now1 = clock.now()

            if now1 - self._last_publish_time > duration3:
                self._diag_pub.publish(msg)
                self._last_publish_time = clock.now()


def main():
    hostname = socket.gethostname()
    import optparse
    parser = optparse.OptionParser(usage='usage: hd_monitor.py [--diag-hostname=cX]')
    parser.add_option('--diag-hostname', dest='diag_hostname',
                      help='Computer name in diagnostics output (ex: "c1")',
                      metavar='DIAG_HOSTNAME',
                      action='store', default=hostname)
    options, args = parser.parse_args()

    home_dir = ''
    if len(args) > 1:
        home_dir = args[1]

    hostname_clean = str.translate(hostname, str.maketrans('-', '_'))
    try:
        rclpy.init()
        node = rclpy.create_node('hd_monitor_%s' % hostname_clean)
    except rclpy.exceptions.NotInitializedException:
        sys.exit(0)

    hd_monitor1 = hd_monitor(hostname, node, options.diag_hostname, home_dir)

    try:
        while rclpy.ok():
            sleep(1)
            hd_monitor1.publish_stats()

    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()

    hd_monitor1.cancel_timers()
    sys.exit(0)


if __name__ == '__main__':
    main()
