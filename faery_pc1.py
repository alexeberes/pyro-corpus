import copy
from datetime import datetime
import os
import pickle
import random

import numpy as np
import merge_sort
from solution import Solution
import constants as Cnsts

import body_builder
import body_mutator
import brain_mutator
from body_parts import *

class FAERYvPyrCor1:
    
    def __init__(self) -> None:
        os.system("rm ./data/robot/robot_fitness*.txt")
        os.system("rm ./data/robot/brain*.nndf")
        os.system("rm ./data/robot/body*.urdf")
        self.parents = {}
        self.next_available_id = 0

        self.generation_size = Cnsts.generation_size
        self.number_of_children = Cnsts.number_of_children
        self.family_filter_size = Cnsts.family_filter_size
        self.random_members = Cnsts.random_members
        self.total_filter_size = self.generation_size - self.random_members

        for parent_num in range(Cnsts.generation_size):
            parent_id = "000" + f"{parent_num:03}" + f"{parent_num:03}" + "000"
            self.parents[parent_id] = Solution(solution_id=parent_id)
            self.next_available_id += 1
        
        self.rng = np.random.default_rng()
        self.max_fitnesses = []

    def evolve(self) -> None:
        self.evaluate(self.parents)

        for generation in range(Cnsts.num_generations):
            os.system("rm ./data/robot/robot_fitness*.txt")
            os.system("rm ./data/robot/brain*.nndf")
            os.system("rm ./data/robot/body*.urdf")
            self.evolve_for_one_generation(generation)
        
        self.show_best()

    def evaluate(self, solutions) -> None:
        for key in solutions:
            solution = solutions[key]
            solution.start_simulation()
        
        for key in solutions:
            solution = solutions[key]
            solution.wait_for_simulation_to_end()
    
    def evolve_for_one_generation(self, generation):
        self.produce_children(generation)
        self.evaluate(self.children)
        self.print()
        self.select(generation)

    def produce_children(self, generation):
        self.children = {}
        for parent in self.parents:
            for child_num in range(self.number_of_children):
                child_id = f"{generation+1:03}" + parent[3:6] + parent[3:6] + f"{child_num:03}"
                child_body_chromosone, child_brain_chromosome, child_bodycons_id = self.mutate(copy.deepcopy(self.parents[parent].genome), child_id)
                child_genome = Genome(child_bodycons_id, child_brain_chromosome, child_body_chromosone)
                self.children[child_id] = Solution(child_id, child_genome)

    def mutate(self, genome_to_mutate, genome_id):
        running = True

        while running:
            try:
                mutated_body_chromosome, new_bodycons_id = body_mutator.mutate(genome_to_mutate.body_chromosome, genome_to_mutate.bodycons_id)

                pyrosim.Start_URDF("./data/robot/body{}.urdf".format(genome_id))

                body_builder.build_body(mutated_body_chromosome)

                pyrosim.End()

                running = False

            except:
                pyrosim.End()
                os.system("rm ./data/robot/body{}.urdf".format(genome_id))
                print("invalid body plan, retrying")
        
        os.system("rm ./data/robot/body{}.urdf".format(genome_id))

        if genome_to_mutate.brain_chromosome is not None:
            mutated_brain_chromosome = brain_mutator.mutate(genome_to_mutate.brain_chromosome, self.rng, genome_to_mutate.brain_chromosome.shape, Cnsts.mutation_rate, Cnsts.mutation_magnitude)
        else:
            mutated_brain_chromosome = None
        
        return mutated_body_chromosome, mutated_brain_chromosome, new_bodycons_id


    def print(self) -> None:
        parent_fitnesses = []
        for key in self.parents:
            parent_fitnesses.append(self.parents[key].fitness)

        child_fitnesses = []
        for key in self.children:
            child_fitnesses.append(self.children[key].fitness)

        print("\np max: {} \t\t c max: {}".format(np.max(parent_fitnesses), np.max(child_fitnesses)))
        print("p mean: {} \t\t c mean: {}\n".format(np.mean(parent_fitnesses), np.mean(child_fitnesses)))
        self.max_fitnesses.append(max(np.max(parent_fitnesses), np.max(child_fitnesses)))

    def select(self, generation) -> None:
        individuals = self.children | self.parents
        sorted_individual_indices = self.sort_individuals(individuals)
        next_generation = {}
        family_counts = {}
        while len(next_generation) < self.total_filter_size:
            top_individual_index = sorted_individual_indices[0]
            top_individual = individuals[top_individual_index]
            top_individual_family1 = top_individual_index[3:6]
            top_individual_family2 = top_individual_index[6:9]
            if top_individual_family1 not in family_counts:
                family_counts[top_individual_family1] = 0
            if top_individual_family2 not in family_counts:
                family_counts[top_individual_family2] = 0
            if family_counts[top_individual_family1] > self.family_filter_size or family_counts[top_individual_family2] > self.family_filter_size:
                sorted_individual_indices.remove(top_individual_index)
            else:
                family_counts[top_individual_family1] += 1
                family_counts[top_individual_family2] += 1
                next_generation[top_individual_index] = top_individual
                sorted_individual_indices.remove(top_individual_index)
        new_members = {}
        for random_member_index in range(self.random_members):
            new_individual_key = self.generation_size + (generation * self.random_members) + random_member_index
            new_id = f"{generation:03}" + f"{new_individual_key:03}" + f"{new_individual_key:03}" + f"{random_member_index:03}"
            new_members[new_id] = Solution(solution_id=new_id)
        self.evaluate(new_members)
        self.parents = next_generation | new_members

    def show_best(self) -> None:
        individuals = self.children | self.parents
        sorted_individual_indices = self.sort_individuals(individuals)
        top_individual_index = sorted_individual_indices[0]
        top_individual = individuals[top_individual_index]

        now = datetime.now()
        date_time_str = now.strftime("%Y-%m-%d.%H_%M_%S_%f")
        fitness_file_name = "./data/output/fitnesses_{}.pylist".format(date_time_str)
        with open(fitness_file_name, "wb") as fp:
            pickle.dump(self.max_fitnesses, fp)
        genome_file_name = "./data/output/genome_{}.pygenome".format(date_time_str)
        with open(genome_file_name, "wb") as fp:
            pickle.dump(top_individual.genome, fp)

    def sort_individuals(self, individuals):
        individual_fitness_dict = {}
        for individual in individuals:
            individual_fitness_dict[individual] = individuals[individual].fitness
        return merge_sort.merge_sort(individual_fitness_dict)
