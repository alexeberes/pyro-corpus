import os
import random
from build_body import *
import simulate_recursive_body
import pybullet as pblt
import constants as Cnsts

for i in range(Cnsts.num_random_bodies):
    solution_id = 0

    body_plan = BodyCons(RandomSizedBodyPiece(),
                         random.randint(1, 3),
                         BodyCons(RandomSizedSensorPiece(),
                                  random.randint(1, 3),
                                  BodyCons(RandomSizedBodyPiece(),
                                           random.randint(1, 3),
                                           BodyCons(RandomSizedSensorPiece(),
                                                    random.randint(1, 3),
                                                    BodyCons(RandomSizedBodyPiece(),
                                                             random.randint(1, 3),
                                                             BodyCons(RandomSizedSensorPiece(),
                                                                      random.randint(1, 3),
                                                                      None))))))

    pyrosim.Start_URDF("./data/robot/body{}.urdf".format(solution_id))

    joint_names = build_body(body_plan)

    pyrosim.End()

    pyrosim.Start_NeuralNetwork("./data/robot/brain{}.nndf".format(solution_id))

    build_brain(body_plan, joint_names)

    pyrosim.End()

    os.system('python simulate_recursive_body.py')