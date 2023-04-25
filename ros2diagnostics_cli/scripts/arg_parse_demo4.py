""" 
- chose option
"""
from argparse import ArgumentParser, Namespace

parser = ArgumentParser()
parser.add_argument(
    "-v",
    "--verbose",
    required=True,
    help="verbose opt",
    type=int,
    choices=[1, 2, 3],
)

args: Namespace = parser.parse_args()

if args.verbose == 1:
    print("mode 1")
elif args.verbose == 2:
    print("mode 2")
elif args.verbose == 3:
    print("mode 3")
