import argparse
from faery_pc1 import FAERYvPyrCor1

parser = argparse.ArgumentParser(
    prog = 'search.py',
    description = 'Performs a search for an optimizied neural network weight set for a simulated robot using FAEry Algorithms')
parser.add_argument('-m', '--method', choices=['1'], default='1')

args = parser.parse_args()

if args.method == '1':
    evolutionary_algorithm = FAERYvPyrCor1()
else:
    exit()


evolutionary_algorithm.evolve()

exit()