from typing import Dict
from bridge_common.vehicle_state import VehicleState
from multimethod import multidispatch
from enum import Enum


class TrafficLightColor(Enum):
    UNKNOWN = 0
    GREEN = 1
    YELLOW = 2
    RED = 3


COLOR_MAP = {
    'GREEN': TrafficLightColor.GREEN,
    'RED': TrafficLightColor.RED,
    'YELLOW': TrafficLightColor.YELLOW,
    'UNKNOWN': TrafficLightColor.UNKNOWN
}


class WorldState:
    @multidispatch
    def __init__(self):
        self._vehicles_states: Dict[str, VehicleState] = dict()
        self._frame = 0
        self._traffic_lights: Dict[str, TrafficLightColor] = dict()

    @__init__.register
    def __init__dict(self, data: dict):
        self.__init__()
        self.from_dict(data)

    @property
    def frame(self) -> int:
        return self._frame

    @property
    def traffic_lights(self):
        return self._traffic_lights

    def set_frame(self, frame):
        self._frame = frame

    def from_dict(self, data):
        self._frame = data['frame']
        for uid, v_data in data['vehicle_state'].items():
            self._vehicles_states[uid] = VehicleState(v_data)
        self._traffic_lights = dict()
        for uid, color_txt in data['traffic_light'].items():
            self._traffic_lights[uid] = COLOR_MAP[color_txt]

    def to_dict(self) -> dict:
        data = dict()
        data['frame'] = self.frame
        data['vehicle_state'] = {
            uid: self.get_vehicle_state(uid).to_dict() for uid in self.vehicle_ids
        }
        data['traffic_light'] = {
            uid: self._traffic_lights[uid].name for uid in self._traffic_lights
        }
        return data

    @property
    def vehicle_ids(self):
        return self._vehicles_states.keys()

    def get_vehicle_state(self, uid: str):
        return self._vehicles_states[uid]

    def set_vehicle_state(self, uid: str, state: VehicleState):
        self._vehicles_states[uid] = state
