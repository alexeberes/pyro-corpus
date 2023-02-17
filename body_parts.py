from __future__ import annotations
import pyrosim_modded.pyrosim_modded as pyrosim
from typing import NamedTuple
from typing import Union
from enum import Enum
import random
from bidict import bidict

VALID_CUBE_ELEMENTS = (1, 6)

class CubeElement(Enum):
    CENTER              = (0, 0, 0)
    FRONT               = (1, 0, 0)
    BACK                = (-1, 0, 0)
    RIGHT               = (0, 1, 0)
    LEFT                = (0, -1, 0)
    TOP                 = (0, 0, 1)
    BOTTOM              = (0, 0, -1)
    FRONT_RIGHT         = (1, 1, 0)
    FRONT_LEFT          = (1, -1, 0)
    FRONT_TOP           = (1, 0, 1)
    FRONT_BOTTOM        = (1, 0, -1)
    BACK_RIGHT          = (-1, 1, 0)
    BACK_LEFT           = (-1, -1, 0)
    BACK_TOP            = (-1, 0, 1)
    BACK_BOTTOM         = (-1, 0, -1)
    RIGHT_TOP           = (0, 1, 1)
    RIGHT_BOTTOM        = (0, 1, -1)
    LEFT_TOP            = (0, -1, 1)
    LEFT_BOTTOM         = (0, -1, -1)
    FRONT_RIGHT_TOP     = (1, 1, 1)
    FRONT_RIGHT_BOTTOM  = (1, 1, -1)
    FRONT_LEFT_TOP      = (1, -1, 1)
    FRONT_LEFT_BOTTOM   = (1, -1, -1)
    BACK_RIGHT_TOP      = (-1, 1, 1)
    BACK_RIGHT_BOTTOM   = (-1, 1, -1)
    BACK_LEFT_TOP       = (-1, -1, 1)
    BACK_LEFT_BOTTOM    = (-1, -1, -1)

OPPOSITE_ELEMENTS = bidict({
    CubeElement.CENTER              :   CubeElement.CENTER,
    CubeElement.FRONT               :   CubeElement.BACK,
    CubeElement.RIGHT               :   CubeElement.LEFT,
    CubeElement.TOP                 :   CubeElement.BOTTOM,
    CubeElement.FRONT_RIGHT         :   CubeElement.BACK_LEFT,
    CubeElement.FRONT_LEFT          :   CubeElement.BACK_RIGHT,
    CubeElement.FRONT_TOP           :   CubeElement.BACK_BOTTOM,
    CubeElement.FRONT_BOTTOM        :   CubeElement.BACK_TOP,
    CubeElement.RIGHT_TOP           :   CubeElement.LEFT_BOTTOM,
    CubeElement.RIGHT_BOTTOM        :   CubeElement.LEFT_TOP,
    CubeElement.FRONT_RIGHT_TOP     :   CubeElement.BACK_LEFT_BOTTOM,
    CubeElement.FRONT_RIGHT_BOTTOM  :   CubeElement.BACK_LEFT_TOP,
    CubeElement.FRONT_LEFT_TOP      :   CubeElement.BACK_RIGHT_BOTTOM,
    CubeElement.FRONT_LEFT_BOTTOM   :   CubeElement.BACK_RIGHT_TOP
})

def get_opposite_cube_element(cube_element: CubeElement):
    try:
        return OPPOSITE_ELEMENTS[cube_element]
    except:
        return OPPOSITE_ELEMENTS.inverse[cube_element][0]

class Axes(Enum):
    X   =   [1, 0, 0]
    Y   =   [0, 1, 0]
    Z   =   [0, 0, 1]

class Position(NamedTuple):
    x:  float
    y:  float
    z:  float

class Dimensions(NamedTuple):
    length: float
    width:  float
    height: float

class BodyCons(NamedTuple):
    body_part:              Union[BodyPart, BodyCons]
    build_specifications:   list[BuildSpecifications]
    next_part:              Union[BodyCons, list[BodyCons], None]=None

class BuildSpecifications(NamedTuple):
    direction_to_build: CubeElement=CubeElement.FRONT
    repitions:      int=1
    axis:           Axes=Axes.X

def create_random_xyz(mins: Union[float, Dimensions, Position], maxes: Union[float, Dimensions, Position]) -> Union[Dimensions, Position]:
    return tuple(mins[index] + (maxes[index] - mins[index]) * random.random() for index in range(3))

def add_xyz(xyz_tuple1: Union[Position, Dimensions, tuple[float, float, float]], xyz_tuple2: Union[Position, Dimensions, tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple(map(lambda var1, var2: var1 + var2, xyz_tuple1, xyz_tuple2))

def scalar_multiplication_xyz(scalar: float, xyz_tuple: Union[Position, Dimensions, tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple(scalar * var for var in xyz_tuple)

def element_wise_multiplication_xyz(xyz_tuple1: Union[Position, Dimensions, tuple[float, float, float]], xyz_tuple2: Union[Position, Dimensions, tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple(map(lambda var1, var2: var1 * var2, xyz_tuple1, xyz_tuple2))

def create_joint(upstream_center: Position, parent_part_size: Dimensions, joint_attachment_element: CubeElement, axis: list[Union[float, int]], current_part_id: int) -> str:
    joint_name = str(current_part_id) + "_" + str(current_part_id + 1)

    joint_attachment_point = add_xyz(
        upstream_center,
        element_wise_multiplication_xyz(
            scalar_multiplication_xyz(
                0.5,
                parent_part_size),
            joint_attachment_element.value)
        )
    
    pyrosim.Send_Joint(
        name=joint_name,
        parent=str(current_part_id),
        child=str(current_part_id + 1),
        type="revolute",
        position=list(joint_attachment_point),
        axis=axis.value)
    
    return joint_name

class BodyPart():

    def __init__(self):
        self.properties = {
            'sensor':   True,
            'motor':    True
        }

    def get_properties(self):
        return self.properties

    def create_body_part(self, upstream_position: Position, attachment_point_on_child: CubeElement, piece_id: int) -> tuple(Position, Dimensions):
        return self.create_body_part(upstream_position, attachment_point_on_child, piece_id)
    
class RandomSizedBodyPiece(BodyPart):
    
    def __init__(self):
        super().__init__()
        self.properties['sensor'] = False

    def create_body_part(self, upstream_position: Position, attachment_point_on_child: CubeElement, piece_id: int) -> tuple(Position, Dimensions):
        size: Dimensions = Dimensions(*create_random_xyz(Dimensions(0.2, 0.2, 0.1), Dimensions(1, 1, 0.6)))

        center: Position = add_xyz(
            upstream_position,
            element_wise_multiplication_xyz(
                scalar_multiplication_xyz(
                    -0.5,
                    size),
                attachment_point_on_child.value))

        pyrosim.Send_Cube(
            name=str(piece_id),
            pos=list(center),
            size=list(size),
            color=('Blue', [0.0, 0.0, 1.0, 1.0]))

        return center, size
    
class RandomSizedSensorPiece(BodyPart):
    
    def __init__(self):
        super().__init__()
        self.properties['sensor'] = True

    def create_body_part(self, upstream_position: Position, attachment_point_on_child: CubeElement, piece_id: int) -> tuple(Position, Dimensions):
        size: Dimensions = Dimensions(*create_random_xyz(Dimensions(0.2, 0.2, 0.1), Dimensions(1, 1, 0.6)))

        center: Position = add_xyz(
            upstream_position,
            element_wise_multiplication_xyz(
                scalar_multiplication_xyz(
                    -0.5,
                    size),
                attachment_point_on_child.value))

        pyrosim.Send_Cube(
            name=str(piece_id),
            pos=list(center),
            size=list(size),
            color=('Green', [0.0, 1.0, 0.0, 1.0]))

        return center, size