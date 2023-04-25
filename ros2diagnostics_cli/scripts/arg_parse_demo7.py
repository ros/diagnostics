""" 
- select argument


arg_parse_demo7 -v
arg_parse_demo7 -vv
arg_parse_demo7 -v -s
usage: arg_parse_demo7.py [-h] [-v | -s] [square]
"""
from argparse import ArgumentParser, Namespace

parser = ArgumentParser()
group = parser.add_mutually_exclusive_group()
parser.add_argument("square", help="square given", type=int, default=0, nargs="?")
group.add_argument("-v", "--verbose", action="count", help="verbose, use -vv for more")
group.add_argument("-s", "--silence", action="store_true", help="no verbose")
args: Namespace = parser.parse_args()
result: int = args.square**2

match args.verbose:
    case 1:
        print("verbose")
    case 2:
        print("more verbose")
    case _:
        print("")




