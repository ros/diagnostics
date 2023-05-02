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


"""_summary_

std_msgs/Header header # for timestamp
builtin_interfaces/Time stamp
    int32 sec
    uint32 nanosec
string frame_id
DiagnosticStatus[] status # an array of components being reported on
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3
byte level
string name
string message
string hardware_id
KeyValue[] values
    string key
    string value
"""
import pathlib
import re
from argparse import ArgumentParser
from enum import IntEnum
from typing import Dict, List, TextIO, Tuple

import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.qos import qos_profile_system_default

TOPIC_DIAGNOSTICS = "/diagnostics"

COLOR_DEFAULT = "\033[39m"
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"

level_to_str_mapping = {
    b"\x00": lambda: ("OK", GREEN + "OK" + COLOR_DEFAULT),
    b"\x01": lambda: ("WARN", YELLOW + "WARN" + COLOR_DEFAULT),
    b"\x02": lambda: ("ERROR", RED + "ERROR" + COLOR_DEFAULT),
    b"\x03": lambda: ("STALE", RED + "STALE" + COLOR_DEFAULT),
}


def convert_level_to_str(level: bytes) -> Tuple[str, str]:
    return level_to_str_mapping[level]()


def open_file_for_output(csv_file: str) -> TextIO:
    base_dir = pathlib.Path(csv_file).parents[0]
    folder_exists = pathlib.Path(base_dir).is_dir()
    if not folder_exists:
        raise Exception(
            f"Folder {csv_file} not exists"
        )  # pylint: disable=[broad-exception-raised]
    f = open(csv_file, "w", encoding="utf-8")
    return f


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
        self.__levels_info = ",".join(levels) if levels is not None else ""
        self.__levels = DiagnosticsParser.map_level_from_name(levels)

    def set_render(self, handler):
        self.__status_render_handler = handler

    def run(self):
        self.register_diagnostics_topic()

    def __filter_level(self, level):
        if not self.__levels:
            return False
        return level not in self.__levels

    @staticmethod
    def map_level_from_name(levels: List[str]) -> List[bytes]:
        b_levels = []
        if levels is None:
            return b_levels

        map_levels = {
            "info": lambda: b_levels.append(b"\x00"),
            "warn": lambda: b_levels.append(b"\x01"),
            "error": lambda: b_levels.append(b"\x02"),
        }

        for level in levels:
            map_levels[level]()
        return b_levels

    @staticmethod
    def render(status: DiagnosticStatus, time_sec, verbose):
        _, level_name = convert_level_to_str(status.level)
        item = f"{status.name}: {level_name}, {status.message}"
        print(item)
        if verbose:
            kv: KeyValue
            for kv in status.values:
                print(f"- {kv.key}={kv.value}")

    def diagnostics_status_handler(self, msg: DiagnosticArray) -> None:
        """Run handler for each DiagnosticStatus in array
        filter status by level, name, node name

        Args:
            msg (DiagnosticArray): _description_
        """
        counter: int = 0
        status: DiagnosticStatus
        print(f"--- time: {msg.header.stamp.sec} ---")
        for status in msg.status:
            if self.__name_filter:
                result = re.search(self.__name_filter, status.name)
                if not result:
                    continue
            if self.__filter_level(status.level):
                continue
            self.__status_render_handler(status, msg.header.stamp.sec, self.__verbose)
            counter += 1

        if not counter:
            print(f"No diagnostic for levels: {self.__levels_info}")

    def register_diagnostics_topic(self):
        """
        create ros node and
        subscribe to /diagnostic topic
        """
        if self.__mode == ParserModeEnum.LIST:
            handler = diagnostic_list_handler
        else:
            handler = self.diagnostics_status_handler

        rclpy.init()
        node = rclpy.create_node("ros2diagnostics_cli_filter")
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
    """Group diagnostics Task by node
    print group data as yaml to stdout

    Args:
        msg (DiagnosticArray): /diagnostics topic message 
        http://docs.ros.org/en/noetic/api/diagnostic_msgs/html/msg/DiagnosticArray.html
    """
    status: DiagnosticStatus
    data: Dict[str, List[str]] = {}
    print(f"--- time: {msg.header.stamp.sec} ---")
    for status in msg.status:
        if ":" in name:
            node, name = status.name.split(":")
        else:
            node = "Aggr"
            name = status.name
        name = name.strip()
        if node in data:
            data[node].append(name)
        else:
            data[node] = [name]

    print(yaml.dump(data))


def add_common_arguments(parser: ArgumentParser):
    """common arguments for csv and show verbs"""
    parser.add_argument("-1", "--once", action="store_true", help="run only once")
    parser.add_argument(
        "-f", "--filter", type=str, help="filter diagnostic status name"
    )
    parser.add_argument(
        "-l",
        "--levels",
        action="append",
        type=str,
        choices=["info", "warn", "error"],
        help="levels to filter, can be multiple times",
    )
