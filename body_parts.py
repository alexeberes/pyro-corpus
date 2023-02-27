from __future__ import annotations
import pyrosim_modded.pyrosim_modded as pyrosim
from typing import NamedTuple
from typing import Union
from enum import Enum
import random
from bidict import bidict
import numpy as np

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
    body_cons_id:           int
    body_part:              Union[BodyPart, BodyCons]
    build_specifications:   list[BuildSpecifications]
    next_body_plans:        Union[dict[CubeElement, BodyCons], None]=None

class BuildSpecifications(NamedTuple):
    direction_to_build: CubeElement=CubeElement.FRONT
    repitions:      int=1
    axis:           Axes=Axes.X

class Genome(NamedTuple):
    bodycons_id:        int
    brain_chromosome:   NeuronWeightMatrix
    body_chromosome:    BodyCons

def create_random_xyz(mins: Union[float, Dimensions, Position], maxes: Union[float, Dimensions, Position]) -> Union[Dimensions, Position]:
    return tuple(mins[index] + (maxes[index] - mins[index]) * random.random() for index in range(3))

def add_xyz(xyz_tuple1: Union[Position, Dimensions, tuple[float, float, float]], xyz_tuple2: Union[Position, Dimensions, tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple(map(lambda var1, var2: var1 + var2, xyz_tuple1, xyz_tuple2))

def scalar_multiplication_xyz(scalar: float, xyz_tuple: Union[Position, Dimensions, tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple(scalar * var for var in xyz_tuple)

def element_wise_multiplication_xyz(xyz_tuple1: Union[Position, Dimensions, tuple[float, float, float]], xyz_tuple2: Union[Position, Dimensions, tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple(map(lambda var1, var2: var1 * var2, xyz_tuple1, xyz_tuple2))

def create_joint(upstream_center: Position, parent_part_size: Dimensions, joint_attachment_element: CubeElement, joint_type:str, axis: list[Union[float, int]], parent_part_id: int, current_part_id: int) -> str:
    joint_name = "" + str(parent_part_id) + "_" + str(current_part_id) + ""

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
        parent=str(parent_part_id),
        child=str(current_part_id),
        type=joint_type,
        position=list(joint_attachment_point),
        axis=axis.value)
    
    return joint_name

class BodyPart():

    def __init__(self):
        self.properties = {
            'sensor'        : True,
            'joint'         : 'revolute',
            'unchangeable'  : False,
            'brain'         : False
        }

    def get_properties(self):
        return self.properties

    def create_body_part(self, upstream_position: Position, attachment_point_on_child: CubeElement, piece_id: int) -> tuple(Position, Dimensions):
        return self.create_body_part(upstream_position, attachment_point_on_child, piece_id)
    
class RandomSizeBodyPiece(BodyPart):
    
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
    
class RandomSizeSensorPiece(BodyPart):
    
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
    

class FixedSizeBodyPiece(BodyPart):
    
    def __init__(self, size: float=1):
        super().__init__()
        self.properties['sensor'] = False
        self.size = size

    def create_body_part(self, upstream_position: Position, attachment_point_on_child: CubeElement, piece_id: int) -> tuple(Position, Dimensions):
        size: Dimensions = Dimensions(self.size, self.size, self.size)

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
    
class FixedSizeSensorPiece(BodyPart):
    
    def __init__(self, size: float=1):
        super().__init__()
        self.properties['sensor'] = True
        self.size = size

    def create_body_part(self, upstream_position: Position, attachment_point_on_child: CubeElement, piece_id: int) -> tuple(Position, Dimensions):
        size: Dimensions = Dimensions(self.size, self.size, self.size)

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
    
class FixedSizeUnmovableBodyPiece(BodyPart):
    
    def __init__(self, size: float=1):
        super().__init__()
        self.properties['sensor'] = False
        self.properties['joint'] = 'fixed'
        self.size = size

    def create_body_part(self, upstream_position: Position, attachment_point_on_child: CubeElement, piece_id: int) -> tuple(Position, Dimensions):
        size: Dimensions = Dimensions(self.size, self.size, self.size)

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
            color=('Purple', [0.5, 0.0, 0.5, 1.0]))

        return center, size
    
class FixedSizeUnmovableSensorPiece(BodyPart):
    
    def __init__(self, size: float=1):
        super().__init__()
        self.properties['sensor'] = True
        self.properties['joint'] = 'fixed'
        self.size = size

    def create_body_part(self, upstream_position: Position, attachment_point_on_child: CubeElement, piece_id: int) -> tuple(Position, Dimensions):
        size: Dimensions = Dimensions(self.size, self.size, self.size)

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
            color=('Orange', [1.0, 0.35, 0.2, 1.0]))

        return center, size
    
class FixedSizedUnchangeableBrain(BodyPart):
    
    def __init__(self, size: float=1):
        super().__init__()
        self.properties['sensor'] = True
        self.properties['unchangeable'] = True
        self.properties['brain'] = True
        self.size = size

    def create_body_part(self, upstream_position: Position, attachment_point_on_child: CubeElement, piece_id: int) -> tuple(Position, Dimensions):
        size: Dimensions = Dimensions(self.size, self.size, self.size)

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
            color=('Black', [0.0, 0.0, 0.0, 1.0]))

        return center, size

class NeuronWeightMatrix():
    
    def __init__(self, sensor_neurons: list[str], motor_neurons: list[str], previous_weights: Union[NeuronWeightMatrix, None] = None) -> None:
        self.shape = (len(sensor_neurons), len(motor_neurons))
        self.sensors = { sensor_neurons[x]: x for x in range(len(sensor_neurons)) }
        self.motors = { motor_neurons[x]: x for x in range(len(motor_neurons)) }
        self.matrix = np.full(self.shape, np.nan)

        if previous_weights is not None:
            generator_sensors = (sensor for sensor in self.sensors if sensor in previous_weights.get_sensors())
            generator_motors = (motor for motor in self.motors if motor in previous_weights.get_motors())

            for sensor in generator_sensors:
                for motor in generator_motors:
                    self_sensor_index = self.sensors[sensor]
                    self_motor_index = self.motors[motor]
                    previous_sensor_index = previous_weights.get_sensors()[sensor]
                    previous_motor_index = previous_weights.get_motors()[motor]
                    self.matrix[self_sensor_index, self_motor_index] = previous_weights.get_weights[previous_sensor_index, previous_motor_index]

        self.matrix[np.isnan(self.matrix)] = np.random.randn(len(self.matrix[np.isnan(self.matrix)]))

    def get_sensors(self):
        return self.sensors
    
    def get_motors(self):
        return self.motors
    
    def get_weights(self):
        return self.matrix
    
    def set_weights(self, weights):
        self.matrix = weights
    
    def get(self, sensor, motor):
        sensor_index = self.sensors[sensor]
        motor_index = self.motors[motor]
        return self.matrix[sensor_index, motor_index]
