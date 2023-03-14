from body_builder import *
from simulation import Simulation
import pickle
import argparse

parser = argparse.ArgumentParser(
    prog = 'simulate_genome.py',
    description = 'Simulates the genome saved in a given file.')
parser.add_argument('-f', '--file')

args = parser.parse_args()

def simulate():
    simulation = Simulation("GUI", 0)
    simulation.run()

if __name__ == '__main__':
    fitness_file_name = args.file

    with open(fitness_file_name, "rb") as fp:
        genome: Genome = pickle.load(fp)

    body_plan = genome.body_chromosome

    solution_id = 0

    psz.Start_URDF("./data/robot/body{}.urdf".format(solution_id))

    joint_names, sensor_parts, abstract_centers = build_body(body_plan)

    psz.end()

    psz.Start_NeuralNetwork("./data/robot/brain{}.nndf".format(solution_id))

    weight_matrix = build_brain(joint_names, sensor_parts)

    psz.end()

    simulate()