""" 
- chose option
"""
from argparse import ArgumentParser, Namespace

parser = ArgumentParser()
parser.add_argument("square", help="square given", type=int, default=0, nargs="?")

args: Namespace = parser.parse_args()
result: int = args.square**2
print(result)

