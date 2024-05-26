from functools import singledispatchmethod
from typing import Tuple
import numpy as np
import math


class AutowareVector:
    @singledispatchmethod
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    @__init__.register
    def __init__dict(self, data: dict):
        self.__init__(float(data['x']), float(data['y']))

    def to_dict(self):
        return {'x': self.x, 'y': self.y}

    @property
    def magnitude(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y)

    @property
    def unit(self) -> 'AutowareVector':
        magnitude = self.magnitude
        return AutowareVector(self.x / magnitude, self.y / magnitude)

    def __add__(self, other: 'AutowareVector'):
        return AutowareVector(self.x + other.x, self.y + other.y)

    def __sub__(self, other: 'AutowareVector'):
        return AutowareVector(self.x - other.x, self.y - other.y)

    def __rmul__(self, rate: float):
        return AutowareVector(rate * self.x, rate * self.y)

    def __iter__(self):
        yield self.x
        yield self.y


class AutowareQuaternion:
    def __init__(self, w: float, x: float, y: float, z: float):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __iter__(self):
        yield self.w
        yield self.x
        yield self.y
        yield self.z


def n2b(x_radius, y_radius, z_radius, b):
    x_matrix = np.array([
        [1, 0, 0],
        [0, np.cos(x_radius), -np.sin(x_radius)],
        [0, np.sin(x_radius), np.cos(x_radius)]
    ])

    y_matrix = np.array([
        [np.cos(y_radius), 0, np.sin(y_radius)],
        [0, 1, 0],
        [-np.sin(y_radius), 0, np.cos(y_radius)]
    ])

    z_matrix = np.array([
        [np.cos(z_radius), -np.sin(z_radius), 0],
        [np.sin(z_radius), np.cos(z_radius), 0],
        [0, 0, 1]
    ])

    n = np.matrix(np.array(b)) * np.matrix(x_matrix) * np.matrix(y_matrix) * np.matrix(z_matrix)
    return n


def transform_to_vrf(x, y, z, theta_radius: float) -> Tuple[float, float, float]:
    ret = n2b(0, 0, theta_radius, np.array([x, y, z]))
    return ret[0, 0], ret[0, 1], ret[0, 2]


class AutowareVehicleShape:
    @singledispatchmethod
    def __init__(self, front_edge_to_center: float, back_edge_to_center: float,
                 left_edge_to_center: float, right_edge_to_center: float, height: float):
        self._front_edge_to_center = front_edge_to_center
        self._back_edge_to_center = back_edge_to_center
        self._left_edge_to_center = left_edge_to_center
        self._right_edge_to_center = right_edge_to_center
        self._height = height

    @__init__.register
    def __init__dict(self, data: dict):
        self.__init__(data['front_edge_to_center'], data['back_edge_to_center'],
                      data['left_edge_to_center'], data['right_edge_to_center'], data['height'])

    def to_dict(self):
        return {
            'front_edge_to_center': self.front_edge_to_center,
            'back_edge_to_center': self.back_edge_to_center,
            'left_edge_to_center': self.left_edge_to_center,
            'right_edge_to_center': self.right_edge_to_center,
            'height': self.height
        }

    @property
    def front_edge_to_center(self):
        return self._front_edge_to_center

    @property
    def back_edge_to_center(self):
        return self._back_edge_to_center

    @property
    def left_edge_to_center(self):
        return self._left_edge_to_center

    @property
    def right_edge_to_center(self):
        return self._right_edge_to_center

    @property
    def width(self):
        return self.left_edge_to_center + self.right_edge_to_center

    @property
    def length(self):
        return self.back_edge_to_center + self.front_edge_to_center

    @property
    def height(self):
        return self._height


class AutowareVehicleState:
    @singledispatchmethod
    def __init__(self, x: float, y: float, v: float, a: float, theta_radians: float, shape: AutowareVehicleShape):
        self.x = x
        self.y = y
        self.v = v
        self.a = a
        self._theta_radians = theta_radians
        self.shape = shape

    @__init__.register
    def __init__dict(self, data: dict):
        self.__init__(float(data['x']), float(data['y']), float(data['v']), float(data['a']),
                      float(data['theta_radians']) if 'theta_radians' in data else math.radians(float(data['theta_degrees'])),
                      AutowareVehicleShape(data['shape']))

    def to_dict(self):
        return {
            'x': self.x,
            'y': self.y,
            'v': self.v,
            'a': self.a,
            'theta_radians': self.theta_radians,
            'shape': self.shape.to_dict()
        }

    @property
    def theta_radians(self):
        return self._theta_radians

    @property
    def theta_degrees(self) -> float:
        return math.degrees(self.theta_radians)

    def set_theta_radians(self, theta_radians):
        self._theta_radians = theta_radians

    def set_theta_degrees(self, theta_degrees):
        self._theta_radians = math.radians(theta_degrees)

    @property
    def forward_vector(self) -> AutowareVector:
        return AutowareVector(math.cos(self.theta_radians), math.sin(self.theta_radians))

    @property
    def front_position(self) -> AutowareVector:
        return self.vehicle_center + self.shape.front_edge_to_center * self.forward_vector

    @property
    def vehicle_center(self) -> AutowareVector:
        return AutowareVector(self.x, self.y)

    @property
    def geometry_center(self) -> AutowareVector:
        offset = (self.shape.back_edge_to_center - self.shape.front_edge_to_center) / 2
        geometry_center = self.vehicle_center - offset * self.forward_vector
        return geometry_center

    @property
    def linear_velocity(self) -> AutowareVector:
        return self.v * self.forward_vector

    @property
    def linear_acceleration(self) -> AutowareVector:
        return AutowareVector(math.cos(self.theta_radians) * self.a, math.sin(self.theta_radians) * self.a)
