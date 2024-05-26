from src.common.types import *
from .apollo_vehicle_state import ApolloVehicleShape
from src.semantic_model.system_state import VehicleShape
import json
import sympy
import math


class ApolloSimulationConfig:
    def __init__(self, config_dict):
        self._config_dict = config_dict

    @property
    def apollo_vehicle_shape(self) -> ApolloVehicleShape:
        shape = ApolloVehicleShape.from_dict(
            self._config_dict['ApolloVehicleShapeList'][self._config_dict['ApolloVehicleShapeIndex']])
        # shape._left_edge_to_center -= 0.5
        # shape._right_edge_to_center -= 0.5
        return shape

    @property
    def vehicle_shape(self) -> VehicleShape:
        return VehicleShape(self.apollo_vehicle_shape.length, self.apollo_vehicle_shape.width)

    @property
    def time_interval(self) -> Number:
        return self._config_dict['time_interval']


APOLLO_SIMULATION_CONFIG = ApolloSimulationConfig(json.load(open('src/simulator/apollo/config.json')))
