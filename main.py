import os
import random
import time
from build_body import *
from generate_body_cons import *
import pybullet as pblt
import constants as Cnsts


solution_id = 0

initial_body_plan = copy.deepcopy(BASE_BODYPLAN)

pyrosim.Start_URDF("./data/robot/body{}.urdf".format(solution_id))

joint_names = build_body(initial_body_plan)

pyrosim.End()

pyrosim.Start_NeuralNetwork("./data/robot/brain{}.nndf".format(solution_id))

build_brain(initial_body_plan, joint_names)

pyrosim.End()

os.system('python simulate_recursive_body.py')

for i in range(Cnsts.num_random_bodies):
    solution_id = 0

    running = True
    while running:
        try:

            mutated_body_plan = run_mutation(initial_body_plan, 20)

            pyrosim.Start_URDF("./data/robot/body{}.urdf".format(solution_id))

            joint_names, abstract_centers = build_body(mutated_body_plan)

            pyrosim.End()

            print(joint_names)
            print(abstract_centers)

            running = False

        except:
            pyrosim.End()
            os.system("rm ./data/robot/body{}.urdf".format(solution_id))
            print("invalid body plan, retrying")

    pyrosim.Start_NeuralNetwork("./data/robot/brain{}.nndf".format(solution_id))

    build_brain(mutated_body_plan, joint_names)

    pyrosim.End()

    print("PREPARING TO SIMULATE")

    time.sleep(0)

    os.system('python simulate_recursive_body.py')

    time.sleep(1)