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

from typing import Dict, List
import yaml

from ros2cli.verb import VerbExtension
from ros2diagnostics.api import DiagnosticsParser

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class ListVerb(VerbExtension):
    """List all diagnostic status items group by node name."""

    def main(self, *, args):
        diagnostic_parser = DiagnosticsParser()
        diagnostic_parser.register_diagnostics_topic(diagnostic_list_handler)


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
