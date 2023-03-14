import os
import random
import time
from body_builder import *
from body_mutator import *
import pybullet as pblt
import constants as Cnsts


solution_id = 0

initial_body_plan = copy.deepcopy(BASE_BODYPLAN)

psz.Start_URDF("./data/robot/body{}.urdf".format(solution_id))

joint_names, sensor_parts, abstract_centers = build_body(initial_body_plan)

psz.end()

psz.Start_NeuralNetwork("./data/robot/brain{}.nndf".format(solution_id))

weight_matrix = build_brain(joint_names, sensor_parts)

psz.end()

os.system('python simulate_recursive_body.py')

for i in range(Cnsts.num_random_bodies):
    solution_id = 0

    running = True

    joint_names = []
    sensor_parts = []

    while running:
        try:

            mutated_body_plan = run_mutator(initial_body_plan, BASE_BODYCONS_ID, 20)

            psz.Start_URDF("./data/robot/body{}.urdf".format(solution_id))

            joint_names, sensor_parts, abstract_centers = build_body(mutated_body_plan)

            psz.end()

            print(joint_names)
            print(abstract_centers)

            running = False

        except:
            psz.end()
            os.system("rm ./data/robot/body{}.urdf".format(solution_id))
            print("invalid body plan, retrying")

    
    psz.Start_NeuralNetwork("./data/robot/brain{}.nndf".format(solution_id))

    weight_matrix = build_brain(joint_names, sensor_parts)

    psz.end()

    print("PREPARING TO SIMULATE")

    time.sleep(0)

    os.system('python simulate_recursive_body.py')

    time.sleep(1)