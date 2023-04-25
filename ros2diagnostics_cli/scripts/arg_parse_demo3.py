""" Add positional argument
position argument:
- required: True/False
- action: boolean
"""
from argparse import ArgumentParser, Namespace

parser = ArgumentParser()
parser.add_argument("square", help="square given number", type=int)
parser.add_argument("-v", "--verbose", action="store_true", required=True, help="verbose opt")
args: Namespace = parser.parse_args()

if args.verbose:
    print(f"verbose mode {args.square**2}")
else:
    print(args.square**2)
