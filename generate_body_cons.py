from __future__ import annotations
import copy
from body_parts import *
import random

BASE_BODYPLAN = BodyCons(FixedSizeBodyPiece(), [BuildSpecifications(CubeElement.FRONT, 1, Axes.Y)], None)

NUMBER_OF_MUTATIONS = 10

BODY_TYPES = [FixedSizeBodyPiece(), FixedSizeSensorPiece(), FixedSizeUnmovableBodyPiece(), FixedSizeUnmovableSensorPiece()]

DIRECTIONS_TO_BUILD = [CubeElement.FRONT, CubeElement.LEFT, CubeElement.TOP, CubeElement.BACK, CubeElement.RIGHT, CubeElement.BOTTOM]

REPETITION_MODIFIERS = [-2, -1, +1, +2]

AXES = [Axes.X, Axes.Y, Axes.Z]

# mutation picking

def pick_mutation(mutation_weighting: list[float], mutation_options: list[function]):
    mutation = None

    mutation_choice_value = random.random()

    cumulative_option_value_sum = 0

    for index, option_value in enumerate(mutation_weighting):
        if mutation_choice_value < option_value:
            mutation = mutation_options[index]
            break
        cumulative_option_value_sum += option_value

    if mutation is None:
        mutation = mutation_options[-1]
        
    return mutation

# body plan mutations

def modify_body_type(current_body_plan: BodyCons):
    new_body_type = current_body_type = current_body_plan.body_part
    while type(new_body_type) is type(current_body_type):
        new_body_type = random.choice(BODY_TYPES)
    
    new_body_plan = BodyCons(new_body_type, current_body_plan.build_specifications, current_body_plan.next_body_plans)
    return new_body_plan

def modify_build_sepcs(current_body_plan: BodyCons):
    spec_mutation = pick_mutation(BUILD_SPEC_MUTATION_WEIGHTING, BUILDSPECS_MUTATIONS)

    new_build_specifications = spec_mutation(current_body_plan.build_specifications[0])

    new_body_plan = BodyCons(current_body_plan.body_part, [new_build_specifications], current_body_plan.next_body_plans)
    return new_body_plan

def modify_next(current_body_plan: BodyCons):
    current_directions = current_body_plan.next_body_plans

    next_direction = random.choice(DIRECTIONS_TO_BUILD)
    
    if current_directions is None or next_direction not in list(current_directions.keys()):
        next_mutation = add_new_next_direction
    
    else:
        next_mutation = change_bodyplan_for_direction

    new_next = next_mutation(current_body_plan.next_body_plans, next_direction)

    new_body_plan = BodyCons(current_body_plan.body_part, current_body_plan.build_specifications, new_next)

    return new_body_plan

# build specification mutations

def change_direction_to_build(current_build_specifications: BuildSpecifications):
    new_direction_to_build = current_direction_to_build = current_build_specifications.direction_to_build
    
    while new_direction_to_build == current_direction_to_build:
        new_direction_to_build = random.choice(DIRECTIONS_TO_BUILD)

    new_build_specifications = BuildSpecifications(new_direction_to_build, current_build_specifications.repitions, current_build_specifications.axis)

    return new_build_specifications

def change_number_of_repetitions(current_build_specifications: BuildSpecifications):
    current_repetitions = current_build_specifications.repitions
    new_repetitions = current_repetitions + random.choice(REPETITION_MODIFIERS)

    while new_repetitions <= 0:
        new_repetitions = current_repetitions + random.choice(REPETITION_MODIFIERS)

    new_build_specification = BuildSpecifications(current_build_specifications.direction_to_build, new_repetitions, current_build_specifications.axis)
    return new_build_specification

def change_axis(current_build_specifications: BuildSpecifications):
    new_axis = current_axis = current_build_specifications.axis

    while new_axis == current_axis:
        new_axis = random.choice(AXES)

    new_build_specifications = BuildSpecifications(current_build_specifications.direction_to_build, current_build_specifications.repitions, new_axis)

    return new_build_specifications

# next body plan mutations

def add_new_next_direction(current_next: BodyCons, direction: CubeElement):
    new_next = copy.copy(current_next)
    if new_next is None:
        new_next = {}
    new_next[direction] = BodyCons(random.choice(BODY_TYPES), [BuildSpecifications(random.choice(DIRECTIONS_TO_BUILD), random.randint(1, 5), random.choice(AXES))], None)

    return new_next

def change_bodyplan_for_direction(current_next: dict[CubeElement, BodyCons], direction: CubeElement):
    mutation = pick_mutation(BODYCONS_MUTATION_WEIGHTING, BODYCONS_MUTATIONS)

    new_next = current_next.copy()

    new_next[direction] = mutation(current_next[direction]) 

    return new_next

# mutation lists

BODYCONS_MUTATIONS = [modify_body_type, modify_body_type, modify_next]

BUILDSPECS_MUTATIONS = [change_direction_to_build, change_number_of_repetitions, change_axis]

NEXT_MUTATIONS = [add_new_next_direction, change_bodyplan_for_direction]

# mutation weights

BODYCONS_MUTATION_WEIGHTING = [0.2, 0.2, 0.6]
BUILD_SPEC_MUTATION_WEIGHTING = [0.1, 0.8, 0.1]
NEXT_MUTATION_WEIGHTING = [0.8, 0.2]

# mutation algorithm

def run_mutation(current_body_plan:BodyCons, number_of_mutations: int):
    mutated_body_plan = BodyCons(current_body_plan.body_part, current_body_plan.build_specifications, current_body_plan.next_body_plans)
    for i in range(number_of_mutations):

        mutation = pick_mutation(BODYCONS_MUTATION_WEIGHTING, BODYCONS_MUTATIONS)

        mutated_body_plan = mutation(mutated_body_plan)

    return mutated_body_plan


if __name__ == '__main__':
    initial_body_plan = copy.deepcopy(BASE_BODYPLAN)
    print(initial_body_plan)


    new_body_plan = run_mutation(initial_body_plan, NUMBER_OF_MUTATIONS)
    print(new_body_plan)