from __future__ import annotations
import pyrosim_modded.pyrosim_modded as pyrosim
from typing import NamedTuple
from body_parts import *

def build_body(body_plan: BodyCons):
    def recursive_builder(body_plan: BodyCons, upstream_position: Position, current_part_id:int):
        create_body_part = body_plan.create_body_part
        repetitions = body_plan.repetitions
        next_part = body_plan.next_part

        if isinstance(next_part, list):
            raise ValueError('Only 1D body plans are currently supported')

        my_center, my_size = create_body_part(upstream_position, CubeElement.BACK, current_part_id)
        repetitions_left = repetitions - 1

        if repetitions_left == 0 and next_part is None:
            return

        joint_position:Position = create_joint(my_center, my_size, CubeElement.FRONT, [0, 1, 0], current_part_id)

        print(joint_position)
        print(my_center)
        print(my_size)
        print()

        if repetitions_left == 0:
            next_body_plan = BodyCons(next_part.create_body_part, next_part.repetitions, next_part.next_part)
            return recursive_builder(next_body_plan, joint_position, current_part_id + 1)
        
        next_body_plan = BodyCons(create_body_part, repetitions_left, next_part)
        return recursive_builder(next_body_plan, joint_position, current_part_id + 1)
        
    recursive_builder(body_plan, Position(0, 0, 10), 0)

body_plan = BodyCons(create_random_sized_body_piece, 4, BodyCons(create_random_sized_sensor_piece, 3, BodyCons(create_random_sized_body_piece, 2, BodyCons(create_random_sized_sensor_piece, 2, None))))

pyrosim.Start_URDF('recursive_body_test.urdf')

build_body(body_plan)

pyrosim.End()