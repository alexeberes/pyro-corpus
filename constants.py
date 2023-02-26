import numpy as np

num_iterations = 150000

sleep_time = 0.001/60

num_generations = 50

motor_joint_range = .2

num_random_bodies = 1

default_fitness = 100

mutation_rate = 0.75
mutation_magnitude = 2

generation_size = 4
number_of_children = 5
family_filter_size = 3
random_members = 3

CPG_magnitude = 2
CPG_period_modifier = 2*np.pi

num_simulations_at_once = 5