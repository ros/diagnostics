from ros2cli.verb import VerbExtension


class WorldVerb(VerbExtension):
    """Prints Hello World on the terminal."""

    def main(self, *, args):
        print("Hello, ROS 2 World!")
