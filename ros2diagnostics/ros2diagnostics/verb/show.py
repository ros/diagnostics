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


from ros2cli.verb import VerbExtension
from ros2diagnostics.api import (
    add_common_arguments,
    DiagnosticsParser,
)

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue



class ShowVerb(VerbExtension):
    """Show diagnostics status item info."""

    def add_arguments(self, parser, cli_name):
        add_common_arguments(parser)
        parser.add_argument(
            '--verbose',
            '-v',
            action='store_true',
            help='Display more info.',
        )

    def main(self, *, args):
        diagnostic_parser = DiagnosticsParser(
            verbose=args.verbose,
            levels=args.levels,
            run_once=args.once,
            name_filter=args.filter,
        )
        diagnostic_parser.run()

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
