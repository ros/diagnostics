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


from typing import TextIO
from ros2cli.verb import VerbExtension
from ros2diagnostics_cli.api import (
    DiagnosticsParser,
    open_file_for_output,
    add_common_arguments,
    ParserModeEnum,
    convert_level_to_str
)
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


class CSVVerb(VerbExtension):
    """export /diagnostics message to csv file"""

    def __init__(self):
        super().__init__()
        self.csv: TextIO = None

    def add_arguments(self, parser, cli_name):
        add_common_arguments(parser)
        parser.add_argument(
            "--output", "-o", type=str, required=True, help="export file full path"
        )

        parser.add_argument(
            "--verbose",
            "-v",
            action="store_true",
            help="export DiagnosticStatus values filed",
        )

    def render(self, status: DiagnosticStatus, time_sec, verbose=False):
        level_name, _ = convert_level_to_str(status.level)
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
                levels=args.levels,
            )
            handler.set_render(self.render)
            handler.run()
        finally:
            if self.csv:
                self.csv.close()
