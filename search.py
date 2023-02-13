import argparse
from ludofae1 import ludoFAE1
from ludofae2 import ludoFAE2
from ludofae3 import ludoFAE3

parser = argparse.ArgumentParser(
    prog = 'search.py',
    description = 'Performs a search for an optimizied neural network weight set for a simulated robot using FAEry Algorithms')
parser.add_argument('-m', '--method', choices=['1', '2', '3'], default='1')

args = parser.parse_args()

if args.method == '1':
    evolutionary_algorithm = ludoFAE1()
elif args.method == '2':
    evolutionary_algorithm = ludoFAE2()
elif args.method == '3':
    evolutionary_algorithm = ludoFAE3()
else:
    exit()


evolutionary_algorithm.evolve()

exit()