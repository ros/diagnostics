from ros2cli.verb import VerbExtension
from ros2diagnostics_cli.api import DiagnosticsParser, ParserModeEnum

class ListVerb(VerbExtension):
    """List all diagnostic status items group by node name"""



    def main(self, *, args):
        diagnostic_parser = DiagnosticsParser(ParserModeEnum.List)
        diagnostic_parser.run()
