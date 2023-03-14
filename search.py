import argparse
from faery_pc1mp import FAERYvPyrCor1MP
from faery_pc1nop import FAERYvPyrCor1NoP
import os
import matplotlib.pyplot as plt
import pickle
import numpy as np

from timeit import default_timer as timer

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog = 'search.py',
        description = 'Performs a search for an optimizied neural network weight set for a simulated robot using FAEry Algorithms')
    parser.add_argument('-m', '--method', choices=['1', '2'], default='1')
    parser.add_argument('-b', '--benchmark', choices=['false', 'true'], default='false')

    args = parser.parse_args()

    if args.method == '1':
        evolutionary_algorithm = FAERYvPyrCor1MP()
        benchmark_runs = 20

    elif args.method == '2':
        evolutionary_algorithm = FAERYvPyrCor1NoP()
        benchmark_runs = 10
    else:
        exit()

    best_genome_file_name = None
    fitnesses_file_name = None
    
    if args.benchmark == 'false':

        start_time = timer()

        best_genome_file_name, fitnesses_file_name = evolutionary_algorithm.evolve()

        end_time = timer()

        elapsed_time = end_time - start_time
        print('\n\nexecution time: ', elapsed_time, 'seconds')

    elif args.benchmark == 'true':
        elapsed_times = []
        for i in range(benchmark_runs):
            start_time = timer()

            best_genome_file_name, fitnesses_file_name = evolutionary_algorithm.evolve()

            end_time = timer()

            elapsed_time = end_time - start_time
            print('\n\nexecution time: ', elapsed_time, ' seconds')
            elapsed_times.append(elapsed_time)
        print('\nmean execution time: ', np.mean(elapsed_times), ' seconds')
        print('\nstd. dev. execution time: ', np.std(elapsed_times), ' seconds')

    cmd_to_send = 'python simulate_genome.py -f "{}"'.format(best_genome_file_name)

    input("Press ENTER to continue to simulation")

    os.system(cmd_to_send)

    with open(fitnesses_file_name, "rb") as fp:
        fitnesses = [0] + pickle.load(fp)

    plt.plot(fitnesses)
    plt.show()

    exit()