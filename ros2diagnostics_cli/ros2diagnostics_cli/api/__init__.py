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
from typing import Dict, List, Tuple, TextIO
import yaml
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.qos import qos_profile_system_default
from argparse import ArgumentParser
import pathlib
from enum import IntEnum

TOPIC_DIAGNOSTICS = "/diagnostics"

COLOR_DEFAULT = "\033[39m"
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"


def open_file_for_output(csv_file) -> TextIO:
    base_dir = pathlib.Path(csv_file).parents[0]
    folder_exists = pathlib.Path(base_dir).is_dir()
    if not folder_exists:
        raise Exception(f"Folder {csv_file} not exists")
    f = open(csv_file, "w", encoding="utf-8")
    return f

class ParserModeEnum(IntEnum):
    List = 1
    CSV = 2
    Show = 3
class DiagnosticsParser:
    def __init__(self, mode: ParserModeEnum, verbose=False, levels=None, run_once=False) -> None:
        self.__status_render_handler = self.render
        self.__mode = mode
        self.__run_once = run_once
        self.__verbose = verbose
        self.__levels_info = ",".join(levels) if levels is not None else ""
        self.__levels = DiagnosticsParser.map_level_from_name(levels)

    def set_render(self, handler):
        self.__status_render_handler = handler

    def run(self):
        self.register_and_parse_diagnostics_topic()

    def __filter_level(self, level):
        if not self.__levels:
            return False
        return level not in self.__levels

    @staticmethod
    def map_level_from_name(levels: List[str]) -> List[bytes]:
        b_levels = []
        if levels is None:
            return b_levels

        for level in levels:
            match level:
                case "info":
                    b_levels.append(b"\x00")
                case "warn":
                    b_levels.append(b"\x01")
                case "error":
                    b_levels.append(b"\x02")
        return b_levels

    

    @staticmethod
    def convert_level_to_str(level) -> Tuple(str, str):
        match level:
            case b"\x00":
                return ("OK", GREEN + "OK" + COLOR_DEFAULT)
            case b"\x01":
                return ("WARN", YELLOW + "WARN" + COLOR_DEFAULT)
            case b"\x02":
                return ("ERROR", RED + "ERROR" + COLOR_DEFAULT)
            case b"\x03":
                return ("STALE", RED + "STALE" + COLOR_DEFAULT)
            case _:
                return ("UNDEFINED", "UNDEFINED")

    def render(self, status: DiagnosticStatus, time_sec, verbose):
        _, level_name = DiagnosticsParser.convert_level_to_str(status.level)
        item = f"{status.name}: {level_name}, {status.message}"
        print(item)
        if verbose:
            kv: KeyValue
            for kv in status.values:
                print(f"- {kv.key}={kv.value}")

    def diagnostics_status_handler(self, msg: DiagnosticArray) -> None:
        counter: int = 0
        status: DiagnosticStatus
        print(f"--- time: {msg.header.stamp.sec} ---")
        for status in msg.status:
            if self.__filter_level(status.level):
                continue
            self.__status_render_handler(status, msg.header.stamp.sec, self.__verbose)
            counter += 1

        if not counter:
            print(f"No diagnostic for levels: {self.__levels_info}")

    def register_and_parse_diagnostics_topic(self):
        match self.__mode:
            case ParserModeEnum.List:
                handler = diagnostic_list_handler
            case _ :
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
        msg (DiagnosticArray): _description_
    """
    status: DiagnosticStatus
    data: Dict[str, List[str]] = {}
    print(f"--- time: {msg.header.stamp.sec} ---")
    for status in msg.status:
        node, name = status.name.split(":")
        name = name.strip()
        if node in data:
            data[node].append(name)
        else:
            data[node] = [name]

    print(yaml.dump(data))

def add_common_arguments(parser: ArgumentParser):
    parser.add_argument("-1", "--once", action="store_true", help="run only once")
