import argparse
from faery_pc1 import FAERYvPyrCor1
import os
import matplotlib.pyplot as plt
import pickle

parser = argparse.ArgumentParser(
    prog = 'search.py',
    description = 'Performs a search for an optimizied neural network weight set for a simulated robot using FAEry Algorithms')
parser.add_argument('-m', '--method', choices=['1'], default='1')

args = parser.parse_args()

if args.method == '1':
    evolutionary_algorithm = FAERYvPyrCor1()
else:
    exit()


best_genome_file_name, fitnesses_file_name = evolutionary_algorithm.evolve()

cmd_to_send = 'python simulate_genome.py -f "{}"'.format(best_genome_file_name)

input("Press ENTER to continue to simulation")

os.system(cmd_to_send)

with open(fitnesses_file_name, "rb") as fp:
    fitnesses = [0] + pickle.load(fp)

plt.plot(fitnesses)
plt.show()

exit()