""" 
- option count
arg_parse_demo6 -v
arg_parse_demo6 -vv
arg_parse_demo6 --verbose --verbose
"""
from argparse import ArgumentParser, Namespace

parser = ArgumentParser()
parser.add_argument(
    "-l", "--levels", action="append", type=str, choices=["info", "warn", "error"], help="levels to filter"
)
args: Namespace = parser.parse_args()

print(args.levels)
