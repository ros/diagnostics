from typing import TextIO
from ros2cli.verb import VerbExtension
from ros2diagnostics_cli.api import (
    DiagnosticsParser,
    open_file_for_output,
    add_common_arguments,
    ParserModeEnum,
)
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


class CSVVerb(VerbExtension):
    """export /diagnostics message to csv file"""

    def __init__(self):
        super().__init__()
        self.csv: TextIO = None

    def add_arguments(self, parser, cli_name):
        add_common_arguments(parser)
        parser.add_argument("--output", "-o", type=str, required=True, help="export file full path")

        parser.add_argument(
            "--verbose",
            "-v",
            action="store_true",
            help="export DiagnosticStatus values filed",
        )

    def render(self, status: DiagnosticStatus, time_sec, verbose=False):
        level_name, _ = DiagnosticsParser.convert_level_to_str(status.level)
        node_name, name = status.name.split(":")
        name = name.strip()
        line = [
            str(time_sec),
            level_name,
            node_name,
            name,
            status.message,
            status.hardware_id,
        ]
        if verbose:
            kv: KeyValue
            for kv in status.values:
                line.append(kv.value)

        s_line = ",".join(line) + "\n"
        print(s_line)
        self.csv.write(s_line)

    def main(self, *, args):
        if args.output:
            try:
                self.csv = open_file_for_output(args.output)
            except Exception as error:
                print(str(error))
                return

        try:
            handler = DiagnosticsParser(
                mode=ParserModeEnum.CSV,
                verbose=args.verbose,
                run_once=args.once,
                name_filter=args.filter,
                levels=args.levels
            )
            handler.set_render(self.render)
            handler.run()
        finally:
            if self.csv:
                self.csv.close()
