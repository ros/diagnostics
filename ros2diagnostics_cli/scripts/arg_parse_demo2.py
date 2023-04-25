from argparse import ArgumentParser, Namespace

parser = ArgumentParser()
parser.add_argument("square", help="square given number", type=int)
args: Namespace = parser.parse_args()
print(args.square**2)
