from ros2cli.verb import VerbExtension
from ros2diagnostics_cli.api import DiagnosticsParser, add_common_arguments, ParserModeEnum


class ShowVerb(VerbExtension):
    """Show diagnostics status item info"""
    def add_arguments(self, parser, cli_name):
        add_common_arguments(parser)
        parser.add_argument(
            "--verbose",
            "-v",
            action="store_true",
            help="Display more info.",
        )

        

    def main(self, *, args):
        diagnostic_parser = DiagnosticsParser(
            mode = ParserModeEnum.Show,
            verbose=args.verbose,
            levels=args.levels,
            run_once=args.once,
            name_filter=args.filter)
        diagnostic_parser.run()
