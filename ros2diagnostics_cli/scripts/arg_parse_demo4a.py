""" 
- chose option
"""
from argparse import ArgumentParser, Namespace

parser = ArgumentParser()
parser.add_argument(
    "-o",
    "--output",
    help="output path",
    type=str
)

args: Namespace = parser.parse_args()

if args.output:
    print(args.output)
