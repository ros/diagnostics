# Copyright 2023 robobe
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the robobe nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from argparse import ArgumentParser
from enum import IntEnum
from pathlib import Path
import re
from typing import Dict, List, TextIO, Tuple

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import rclpy
from rclpy.qos import qos_profile_system_default
from osrf_pycommon.terminal_color import format_color

import yaml

TOPIC_DIAGNOSTICS = '/diagnostics'

level_to_str_mapping = {
    DiagnosticStatus.OK: ('OK', format_color("@{greenf}OK@{reset}.")),
    DiagnosticStatus.WARN: ('WARN', format_color("@{yellowf}WARN@{reset}.")),
    DiagnosticStatus.ERROR: ('ERROR', format_color("@{redf}ERROR@{reset}.")),
    DiagnosticStatus.STALE: ('STALE', format_color("@{redf}STALE@{reset}.")),
}


def convert_level_to_str(level: bytes) -> Tuple[str, str]:
    return level_to_str_mapping[level]()


def open_file_for_output(csv_file: str) -> TextIO:
    csv_path = Path(csv_file)
    if not csv_path.parent.is_dir():
        raise FileNotFoundError(
            'Cannot write file, directory does not exist'
        )
    return csv_path.open('w', encoding='utf-8')


class ParserModeEnum(IntEnum):
    LIST = 1
    CSV = 2
    SHOW = 3


class DiagnosticsParser:
    def __init__(
        self,
        mode: ParserModeEnum,
        verbose=False,
        levels=None,
        run_once=False,
        name_filter=None,
    ) -> None:
        self.__name_filter = name_filter
        self.__status_render_handler = DiagnosticsParser.render
        self.__mode = mode
        self.__run_once = run_once
        self.__verbose = verbose
        self.__levels_info = ','.join(levels) if levels is not None else ''
        self.__levels = DiagnosticsParser.map_level_from_name(levels)

    def set_render(self, handler):
        self.__status_render_handler = handler

    def run(self):
        self.register_diagnostics_topic()

    def __filter_level(self, level):
        return False if not self.__levels else level not in self.__levels

    @staticmethod
    def map_level_from_name(levels: List[str]) -> List[bytes]:
        b_levels = []
        if levels is None:
            return b_levels

        map_levels = {
            'info': lambda: b_levels.append(b'\x00'),
            'warn': lambda: b_levels.append(b'\x01'),
            'error': lambda: b_levels.append(b'\x02'),
        }

        for level in levels:
            map_levels[level]()
        return b_levels

    @staticmethod
    def render(status: DiagnosticStatus, time_sec, verbose):
        _, level_name = convert_level_to_str(status.level)
        item = f'{status.name}: {level_name}, {status.message}'
        print(item)
        if verbose:
            kv: KeyValue
            for kv in status.values:
                print(f'- {kv.key}={kv.value}')

    def diagnostics_status_handler(self, msg: DiagnosticArray) -> None:
        """
        Filter DiagnosticStatus by level, name and node name.

        Args:
        ----
            msg (DiagnosticArray): _description_

        """
        counter: int = 0
        status: DiagnosticStatus
        print(f'--- time: {msg.header.stamp.sec} ---')
        for status in msg.status:
            if self.__name_filter:
                result = re.search(self.__name_filter, status.name)
                if not result:
                    continue
            if self.__filter_level(status.level):
                continue
            self.__status_render_handler(
                status, msg.header.stamp.sec, self.__verbose)
            counter += 1

        if not counter:
            print(f'No diagnostic for levels: {self.__levels_info}')

    def register_diagnostics_topic(self):
        """Create ros node and subscribe to /diagnostic topic."""
        if self.__mode == ParserModeEnum.LIST:
            handler = diagnostic_list_handler
        else:
            handler = self.diagnostics_status_handler

        rclpy.init()
        node = rclpy.create_node('ros2diagnostics_cli_filter')
        node.create_subscription(
            DiagnosticArray,
            TOPIC_DIAGNOSTICS,
            handler,
            qos_profile=qos_profile_system_default,
        )
        try:
            if self.__run_once:
                rclpy.spin_once(node)
            else:
                rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


def diagnostic_list_handler(msg: DiagnosticArray) -> None:
    """
    Print group data as yaml to stdout.

    Args:
    ----
        msg (DiagnosticArray): /diagnostics topic message

    """
    status: DiagnosticStatus
    data: Dict[str, List[str]] = {}
    print(f'--- time: {msg.header.stamp.sec} ---')
    for status in msg.status:
        if ':' in status.name:
            node, name = status.name.split(':')
        else:
            node = '---'
            name = status.name
        name = name.strip()
        if node in data:
            data[node].append(name)
        else:
            data[node] = [name]

    print(yaml.dump(data))


def add_common_arguments(parser: ArgumentParser):
    """Add common arguments for csv and show verbs."""
    parser.add_argument(
        '-1',
        '--once',
        action='store_true',
        help='run only once'
    )
    parser.add_argument(
        '-f'
        '--filter',
        type=str,
        help='filter by diagnostic status name'
    )
    parser.add_argument(
        '-l',
        '--levels',
        action='append',
        type=str,
        choices=['info', 'warn', 'error'],
        help='levels to filter, can be multiple times',
    )
