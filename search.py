import argparse
from faery_pc1mp import FAERYvPyrCor1MP
from faery_pc1nop import FAERYvPyrCor1NoP
import os
import matplotlib.pyplot as plt
import pickle

from timeit import default_timer as timer

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog = 'search.py',
        description = 'Performs a search for an optimizied neural network weight set for a simulated robot using FAEry Algorithms')
    parser.add_argument('-m', '--method', choices=['1', '2'], default='1')

    args = parser.parse_args()

    if args.method == '1':
        evolutionary_algorithm = FAERYvPyrCor1MP()
    elif args.method == '2':
        evolutionary_algorithm = FAERYvPyrCor1NoP()
    else:
        exit()

    start_time = timer()

    best_genome_file_name, fitnesses_file_name = evolutionary_algorithm.evolve()

    end_time = timer()
    
    elapsed_time = end_time - start_time
    print('\n\nexecution time: ', elapsed_time, 'seconds')

    cmd_to_send = 'python simulate_genome.py -f "{}"'.format(best_genome_file_name)

    input("Press ENTER to continue to simulation")

    os.system(cmd_to_send)

    with open(fitnesses_file_name, "rb") as fp:
        fitnesses = [0] + pickle.load(fp)

    plt.plot(fitnesses)
    plt.show()

    exit()