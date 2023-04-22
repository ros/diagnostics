from ros2cli.verb import VerbExtension
from ros2diagnostics_cli.api import get_hello_world, get_hello_world_leet


class WorldVerb(VerbExtension):
    """Prints Hello World on the terminal."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "--leet",
            "-l",
            action="store_true",
            help="Display the message in 'l33t' form.",
        )

    def main(self, *, args):
        message = get_hello_world() if not args.leet else get_hello_world_leet()
        print(message)
