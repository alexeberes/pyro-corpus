import numpy as np
import pyrosim_modded.pyrosim_modded as pyrosim
import body_builder
import body_mutator
import brain_mutator
from body_parts import *
import os
import time
import constants as Cnsts
import warnings

class Solution:
    
    def __init__(self, solution_id = 0, genome = None) -> None:
        self.solution_id = solution_id

        self.genome = genome
        if self.genome is None:
            self.genome = Genome(body_mutator.BASE_BODYCONS_ID , None, body_mutator.BASE_BODYPLAN)

        self.mutation_rate = Cnsts.mutation_rate
        self.mutation_magnitude = Cnsts.mutation_magnitude

    def start_simulation(self, pybullet_method = "DIRECT") -> None:
        self.generate_body()
        self.generate_brain()
        os.system("python simulate.py {} {} &".format(pybullet_method, self.solution_id))

    def wait_for_simulation_to_end(self) -> None:
        time.sleep(0.02)
        fitness_file_name = "./data/robot/robot_fitness{}.txt".format(self.solution_id)
        timeout = time.time() + 60 * 10
        while not os.path.exists(fitness_file_name):
            time.sleep(0.02)
            if time.time() > timeout:
                warnings.warn("Warning: simulation timed out")
                os.system("rm {}".format(fitness_file_name))
                self.fitness = Cnsts.default_fitness
                return
        with open(fitness_file_name, 'r') as f:
            fitness = f.read()
            self.fitness = float(fitness)
            f.close()
        time.sleep(0.02)
        os.system("rm {}".format(fitness_file_name))

    def generate_body(self) -> None:
        running = True

        joint_names = []
        sensor_parts = []

        while running:
            try:

                mutated_body_plan, mutated_bodycons_id = body_mutator.mutate(self.genome.body_chromosome, self.genome.bodycons_id)

                pyrosim.Start_URDF("./data/robot/body{}.urdf".format(self.solution_id))

                joint_names, sensor_parts, abstract_centers = body_builder.build_body(mutated_body_plan)

                pyrosim.End()

                self.genome = Genome(mutated_bodycons_id, self.genome.brain_chromosome, mutated_body_plan)

                running = False

            except:
                pyrosim.End()
                os.system("rm ./data/robot/body{}.urdf".format(self.solution_id))
                
        self.joint_names = joint_names
        self.sensor_parts = sensor_parts

    def generate_brain(self) -> None:
        pyrosim.Start_NeuralNetwork("./data/robot/brain{}.nndf".format(self.solution_id))

        weight_matrix = body_builder.build_brain(self.joint_names, self.sensor_parts)

        pyrosim.End()

        self.weight_matrix = weight_matrix
