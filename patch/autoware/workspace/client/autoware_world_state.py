from enum import Enum
from typing import Dict
from client.autoware_vehicle_state import AutowareVehicleState
from typing import Mapping, Optional
from types import MappingProxyType


class TrafficLightColor(Enum):
    UNKNOWN = 0
    GREEN = 1
    YELLOW = 2
    RED = 3

class VehicleMode(Enum):
    AUTOPILOT = 0
    STATIC = 1


COLOR_MAP = {
    'GREEN': TrafficLightColor.GREEN,
    'RED': TrafficLightColor.RED,
    'YELLOW': TrafficLightColor.YELLOW,
    'UNKNOWN': TrafficLightColor.UNKNOWN
}


class AutowareWorldModel:
    def __init__(self, data=None):
        self.vehicle_states: Dict[str, AutowareVehicleState] = dict()
        self.vehicle_modes: Dict[str, VehicleMode] = dict()
        self._frame = 0
        self._traffic_lights: Dict[str, TrafficLightColor] = dict()
        if data is not None:
            self.from_dict(data)

    @property
    def frame(self) -> int:
        return self._frame

    @property
    def traffic_lights(self) -> Mapping[str, TrafficLightColor]:
        return MappingProxyType(self._traffic_lights)

    @property
    def autopilot_ids(self):
        return sorted([uid for uid, mode in self.vehicle_modes.items() if mode == VehicleMode.AUTOPILOT])

    def set_frame(self, frame):
        self._frame = frame

    def from_dict(self, data):
        self._frame = data['frame']
        for uid, v_data in data['vehicle_state'].items():
            self.vehicle_states[uid] = AutowareVehicleState(v_data)
        for uid, mode_txt in data['vehicle_mode'].items():
            self.vehicle_modes[uid] = VehicleMode[mode_txt]
        self._traffic_lights = dict()
        for uid, color_txt in data['traffic_light'].items():
            self._traffic_lights[uid] = COLOR_MAP[color_txt]

    def to_dict(self) -> dict:
        data = dict()
        data['frame'] = self.frame
        data['vehicle_state'] = {
            uid: self.get_vehicle_state(uid).to_dict() for uid in self.vehicle_ids
        }
        data['vehicle_mode'] = {
            uid: self.get_vehicle_mode(uid).name for uid in self.vehicle_modes
        }
        data['traffic_light'] = {
            uid: self._traffic_lights[uid].name for uid in self._traffic_lights
        }
        return data

    def copy(self):
        return AutowareWorldModel(self.to_dict())

    @property
    def vehicle_ids(self):
        return self.vehicle_states.keys()

    def get_vehicle_state(self, uid: str):
        return self.vehicle_states[uid]
    
    def get_vehicle_mode(self, uid: str):
        return self.vehicle_modes[uid]

    def set_vehicle_state(self, uid: str, state: AutowareVehicleState, mode: Optional[VehicleMode] = None):
        self.vehicle_states[uid] = state
        if mode is not None:
            self.vehicle_modes[uid] = mode
        assert self.vehicle_states.keys() == self.vehicle_modes.keys()

    def set_traffic_light_state(self, uid: str, state: TrafficLightColor):
        self._traffic_lights[uid] = state

    def clear_traffic_light(self):
        self._traffic_lights = dict()
