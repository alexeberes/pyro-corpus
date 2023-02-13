from __future__ import annotations
import pyrosim_modded.pyrosim_modded as pyrosim
from typing import NamedTuple
from body_parts import *

def build_body(body_plan: BodyCons):
    def build_body_recursively(body_plan: BodyCons, upstream_position: Position, current_part_id:int):
        body_part: BodyPart = body_plan.body_part
        repetitions = body_plan.repetitions
        next_part = body_plan.next_part

        if isinstance(next_part, list):
            raise ValueError('Only 1D body plans are currently supported')

        my_center, my_size = body_part.create_body_part(
            upstream_position=upstream_position,
            child_attachment_point=CubeElement.BACK,
            piece_id=current_part_id)
        
        repetitions_left = repetitions - 1

        if repetitions_left == 0 and next_part is None:
            return

        joint_position: Position = create_joint(
            upstream_center=my_center,
            parent_part_size=my_size,
            joint_attachment_element=CubeElement.FRONT,
            axis=[0, 1, 0],
            current_part_id=current_part_id)

        if repetitions_left == 0:
            next_body_plan = BodyCons(
                body_part=next_part.body_part,
                repetitions=next_part.repetitions,
                next_part=next_part.next_part)
            return build_body_recursively(body_plan=next_body_plan,
                                          upstream_position=(0, 0, 0),
                                          current_part_id=current_part_id + 1)
        
        next_body_plan = BodyCons(body_part=body_part,
                                  repetitions=repetitions_left,
                                  next_part=next_part)
        return build_body_recursively(body_plan=next_body_plan,
                                      upstream_position=(0, 0, 0),
                                      current_part_id=current_part_id + 1)
        
    build_body_recursively(body_plan=body_plan,
                           upstream_position=Position(0, 0, 2),
                           current_part_id=0)

body_plan = BodyCons(RandomSizedBodyPiece(), 4, BodyCons(RandomSizedSensorPiece(), 3, BodyCons(RandomSizedBodyPiece(), 2, BodyCons(RandomSizedSensorPiece(), 2, None))))

pyrosim.Start_URDF('recursive_body_test.urdf')

build_body(body_plan)

pyrosim.End()