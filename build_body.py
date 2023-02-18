from __future__ import annotations
import pyrosim_modded.pyrosim_modded as pyrosim
from typing import NamedTuple
from body_parts import *
import random


def build_brain(body_plan: BodyCons, joint_names: list[str]):
    sensor_neurons = []
    motor_neurons = []

    def build_brain_recursively(body_plan: BodyCons, current_joint_id: int,  current_part_id: int):
        body_part: BodyPart                             = body_plan.body_part
        build_specifications: list[BuildSpecifications] = body_plan.build_specifications
        next_body_plans                                 = body_plan.next_body_plans

        current_specification: BuildSpecifications = build_specifications[0]

        repetitions = current_specification.repitions
        axis = current_specification.axis

        if body_part.get_properties()['sensor'] == True:
            sensor_neuron_name = "SensorNeuron" + str(current_part_id)
            pyrosim.Send_Sensor_Neuron(name=sensor_neuron_name, linkName=str(current_part_id))
            sensor_neurons.append(sensor_neuron_name)
        
        repetitions_left = repetitions - 1

        if repetitions_left <= 0 and next_body_plans is None:
            return
        
        joint_name = joint_names[current_joint_id]

        motor_neuron_name = "MotorNeuron" + joint_name
        pyrosim.Send_Motor_Neuron(name=motor_neuron_name, jointName=joint_name)
        motor_neurons.append(motor_neuron_name)

        if repetitions_left <= 0:
            for direction in next_body_plans:
                next_body_plan = next_body_plans[direction]
                return build_brain_recursively(body_plan=next_body_plan,
                                                            current_joint_id=current_joint_id + 1,
                                                            current_part_id=current_part_id + 1)
       
        new_build_specifications = [BuildSpecifications(repitions=repetitions_left, axis=axis)]
        
        next_body_plan = BodyCons(body_part=body_part,
                                  build_specifications=new_build_specifications,
                                  next_body_plans=next_body_plans)
        return  build_brain_recursively(body_plan=next_body_plan,
                                                        current_joint_id=current_joint_id + 1,
                                                        current_part_id=current_part_id + 1)
        
    build_brain_recursively(body_plan=body_plan,
                           current_joint_id=0,
                           current_part_id=0)
    
    for sensor_neuron in sensor_neurons:
        for motor_neuron in motor_neurons:
            pyrosim.Send_Synapse(
                sourceNeuronName=sensor_neuron,
                targetNeuronName=motor_neuron,
                weight=-1 + 2 * random.random()
            )


def build_body(body_plan: BodyCons):
    # TODO keep track of increases to current_part_id in downstream chains
    def build_body_recursively(body_plan: BodyCons, upstream_position: Position, upstream_cube_element: CubeElement, current_part_id: int):
        body_part: BodyPart                             = body_plan.body_part
        build_specifications: list[BuildSpecifications] = body_plan.build_specifications
        next_body_plans                                 = body_plan.next_body_plans

        current_specification: BuildSpecifications = build_specifications[0]

        direction_to_build = current_specification.direction_to_build
        repetitions = current_specification.repitions
        axis = current_specification.axis

        my_center, my_size = body_part.create_body_part(
            upstream_position=upstream_position,
            attachment_point_on_child=get_opposite_cube_element(upstream_cube_element),
            piece_id=current_part_id)
        
        repetitions_left = repetitions - 1

        if repetitions_left <= 0 and next_body_plans is None:
            return []

        joint_name: str = create_joint(
            upstream_center=my_center,
            parent_part_size=my_size,
            joint_attachment_element=direction_to_build,
            axis=axis,
            current_part_id=current_part_id)
        
        joint_names = [joint_name]

        if repetitions_left <= 0:
            for direction in next_body_plans:
                print(1)
                next_body_plan = next_body_plans[direction]
                joint_names + build_body_recursively(body_plan=next_body_plan,
                                                            upstream_position=(0, 0, 0),
                                                            upstream_cube_element=direction,
                                                            current_part_id=current_part_id + 1)
            return joint_names
        
        new_build_specifications = [BuildSpecifications(direction_to_build=direction_to_build, repitions=repetitions_left, axis=axis)]
        
        next_body_plan = BodyCons(body_part=body_part,
                                  build_specifications=new_build_specifications,
                                  next_body_plans=next_body_plans)
        return  joint_names + build_body_recursively(body_plan=next_body_plan,
                                      upstream_position=(0, 0, 0),
                                      upstream_cube_element=direction_to_build,
                                      current_part_id=current_part_id + 1)

    return build_body_recursively(body_plan=body_plan,
                           upstream_position=Position(-5, 0, 2),
                           upstream_cube_element=CubeElement.CENTER,
                           current_part_id=0)


body_plan = BodyCons(RandomSizedBodyPiece(), [BuildSpecifications(CubeElement.FRONT, 3, Axes.X)],
                     {CubeElement.LEFT: BodyCons(RandomSizedSensorPiece(), [BuildSpecifications(CubeElement.LEFT, 2, Axes.X)], None),
                      CubeElement.RIGHT: BodyCons(RandomSizedBodyPiece(), [BuildSpecifications(CubeElement.RIGHT, 6, Axes.X)], None),
                      CubeElement.TOP: BodyCons(RandomSizedBodyPiece(), [BuildSpecifications(CubeElement.TOP, 3, Axes.X)], None)})

solution_id = 0

pyrosim.Start_URDF("./data/robot/body{}.urdf".format(solution_id))

joint_names = build_body(body_plan)

pyrosim.End()

pyrosim.Start_NeuralNetwork("./data/robot/brain{}.nndf".format(solution_id))

build_brain(body_plan, joint_names)

pyrosim.End()