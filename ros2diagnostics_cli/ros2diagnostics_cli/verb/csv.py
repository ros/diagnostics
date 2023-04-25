import click
from ros2cli.verb import VerbExtension
from ros2diagnostics_cli.api import get_hello_world, get_hello_world_leet


class CSVVerb(VerbExtension):
    def add_arguments(self, parser, cli_name):
        parser.add_argument("--output", "-o", type=str, help="export file to file")

    def main(self, *, args):
        if args.output:
            print(args.output)
