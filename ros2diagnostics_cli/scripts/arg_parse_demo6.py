""" 
- option count
arg_parse_demo6 -v
arg_parse_demo6 -vv
arg_parse_demo6 --verbose --verbose
"""
from argparse import ArgumentParser, Namespace

parser = ArgumentParser()
parser.add_argument("square", help="square given", type=int, default=0, nargs="?")
parser.add_argument("-v", "--verbose", action="count", help="verbose, use -vv for more")
args: Namespace = parser.parse_args()
result: int = args.square**2

if args.verbose == 1:
    print("verbose")
elif args.verbose == 2:
    print("more verbose")
elif args.verbose > 2:
    print("no extra verbode option max -vv")
else:
    print("no verbose")



