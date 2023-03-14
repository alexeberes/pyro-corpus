import numpy as np
import pyrosim_z as psz
import body_builder
import body_mutator
import brain_mutator
from body_parts import *
import os
import time
import constants as Cnsts
import warnings
from simulation import Simulation

class Solution:
    
    def __init__(self, solution_id = 0, genome = None) -> None:
        self.solution_id = solution_id

        self.genome = genome
        if self.genome is None:
            self.genome = Genome(body_mutator.BASE_BODYCONS_ID , None, body_mutator.BASE_BODYPLAN)

        self.mutation_rate = Cnsts.mutation_rate
        self.mutation_magnitude = Cnsts.mutation_magnitude

    def start_simulation(self, pybullet_method = "DIRECT") -> None:
        simulation = Simulation(pybullet_method, self.solution_id)
        simulation.run()
        return simulation.get_fitness()
    
    def set_fitness(self, fitness):
        self.fitness = fitness

    def generate_body(self) -> None:
        running = True

        joint_names = []
        sensor_parts = []

        while running:
            try:

                mutated_body_plan, mutated_bodycons_id = body_mutator.mutate(self.genome.body_chromosome, self.genome.bodycons_id)

                psz.Start_URDF("./data/robot/body{}.urdf".format(self.solution_id))

                joint_names, sensor_parts, abstract_centers = body_builder.build_body(mutated_body_plan)

                psz.end()

                self.genome = Genome(mutated_bodycons_id, self.genome.brain_chromosome, mutated_body_plan)

                running = False

            except:
                psz.end()
                os.system("rm ./data/robot/body{}.urdf".format(self.solution_id))
                
        self.joint_names = joint_names
        self.sensor_parts = sensor_parts

    def generate_brain(self) -> None:
        psz.Start_NeuralNetwork("./data/robot/brain{}.nndf".format(self.solution_id))

        weight_matrix = body_builder.build_brain(self.joint_names, self.sensor_parts)

        psz.end()

        self.weight_matrix = weight_matrix
        self.genome = Genome(self.genome.bodycons_id, self.weight_matrix, self.genome.body_chromosome)
