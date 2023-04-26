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

        parser.add_argument(
            "-l",
            "--levels",
            action="append",
            type=str,
            choices=["info", "warn", "error"],
            help="levels to filter, can be multiple times",
        )

    def main(self, *, args):
        diagnostic_parser = DiagnosticsParser(
            mode = ParserModeEnum.Show,
            verbose=args.verbose,
                                              levels=args.levels,
                                              run_once=args.once)
        diagnostic_parser.run()
