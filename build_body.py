from __future__ import annotations
import warnings
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
    abstract_centers = []

    def build_body_recursively(body_plan: BodyCons, upstream_position: Position, upstream_cube_element: CubeElement, parrent_part_id: int, current_part_id: int, parent_center: Union[Position, None]=None, parent_size: Union[Dimensions, None]=None, parent_abstract_position: Union[Position, None]=None):
        body_part: BodyPart                             = body_plan.body_part
        build_specifications: list[BuildSpecifications] = body_plan.build_specifications
        next_body_plans                                 = body_plan.next_body_plans

        current_specification: BuildSpecifications = build_specifications[0]

        direction_to_build = current_specification.direction_to_build
        repetitions = current_specification.repitions
        axis = current_specification.axis

        if parrent_part_id == -1:
            joint_name = []
            my_abstract_position = Position(0, 0, 0)
        else:
            my_abstract_position = Position(*add_xyz(parent_abstract_position, direction_to_build.value))
        
        if my_abstract_position in abstract_centers:
            warnings.warn("Center already in design, cannot build")
            # TODO: skip this cube but continue on building
            # TODO: also need to update build brain method
        
            joint_name: list[str] = [create_joint(
                upstream_center=parent_center,
                parent_part_size=parent_size,
                joint_attachment_element=direction_to_build,
                joint_type=body_part.properties['joint'],
                axis=axis,
                parent_part_id=parrent_part_id,
                current_part_id=current_part_id)]

        my_center, my_size = body_part.create_body_part(
            upstream_position=upstream_position,
            attachment_point_on_child=get_opposite_cube_element(direction_to_build),
            piece_id=current_part_id)
        
        
        abstract_centers.append(my_abstract_position)
        
        repetitions_left = repetitions - 1

        if repetitions_left <= 0 and next_body_plans is None:
            return joint_name
        
        joint_names = [] + joint_name

        if repetitions_left <= 0:
            next_parent_id = current_part_id
            for direction in next_body_plans:
                next_body_plan = next_body_plans[direction]
                joint_names += build_body_recursively(body_plan=next_body_plan,
                                                            upstream_position=(0, 0, 0),
                                                            upstream_cube_element=direction,
                                                            parrent_part_id=next_parent_id,
                                                            current_part_id=current_part_id + 1,
                                                            parent_center=my_center,
                                                            parent_size=my_size,
                                                            parent_abstract_position=my_abstract_position)
                last_joint_name = joint_names[-1]
                last_link = last_joint_name.split('_')[1]
                current_part_id = int(last_link)
            return joint_names
        
        new_build_specifications = [BuildSpecifications(direction_to_build=direction_to_build, repitions=repetitions_left, axis=axis)]
        
        next_body_plan = BodyCons(body_part=body_part,
                                  build_specifications=new_build_specifications,
                                  next_body_plans=next_body_plans)
        return  joint_names + build_body_recursively(body_plan=next_body_plan,
                                      upstream_position=(0, 0, 0),
                                      upstream_cube_element=direction_to_build,
                                      parrent_part_id=current_part_id,
                                      current_part_id=current_part_id + 1,
                                      parent_center=my_center,
                                      parent_size=my_size,
                                      parent_abstract_position=my_abstract_position)

    return build_body_recursively(body_plan=body_plan,
                           upstream_position=Position(-15, 10, 5),
                           upstream_cube_element=CubeElement.CENTER,
                           parrent_part_id=-1,
                           current_part_id=0), abstract_centers


if __name__ == '__main__':
    body_plan = BodyCons(FixedSizeBodyPiece(), [BuildSpecifications(CubeElement.FRONT, 3, Axes.X)],
                        {CubeElement.TOP: BodyCons(FixedSizeSensorPiece(), [BuildSpecifications(CubeElement.TOP, 3, Axes.Z)],
                                                    {CubeElement.BACK: BodyCons(FixedSizeUnmovableBodyPiece(), [BuildSpecifications(CubeElement.BACK, 5, Axes.X)],None),
                                                    CubeElement.RIGHT: BodyCons(FixedSizeUnmovableSensorPiece(), [BuildSpecifications(CubeElement.RIGHT, 2, Axes.Y)], None)}),
                        CubeElement.RIGHT: BodyCons(FixedSizeUnmovableSensorPiece(), [BuildSpecifications(CubeElement.RIGHT, 2, Axes.Y)], None),
                        CubeElement.LEFT: BodyCons(FixedSizeSensorPiece(), [BuildSpecifications(CubeElement.LEFT, 2, Axes.Y)], 
                                                    {CubeElement.TOP:BodyCons(FixedSizeUnmovableBodyPiece(), [BuildSpecifications(CubeElement.TOP, 3, Axes.Z)],None)}),
                        CubeElement.FRONT: BodyCons(FixedSizeSensorPiece(), [BuildSpecifications(CubeElement.FRONT, 2, Axes.X)], None),
                        })

    solution_id = 0

    pyrosim.Start_URDF("./data/robot/body{}.urdf".format(solution_id))

    joint_names, abstract_centers = build_body(body_plan)

    pyrosim.End()

    print(joint_names)
    print(abstract_centers)

    pyrosim.Start_NeuralNetwork("./data/robot/brain{}.nndf".format(solution_id))

    build_brain(body_plan, joint_names)

    pyrosim.End()