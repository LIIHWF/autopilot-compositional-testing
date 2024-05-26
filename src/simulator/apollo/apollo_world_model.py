from src.common.types import *
from .apollo_vehicle_state import ApolloVehicleState


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


class ApolloWorldModel:
    def __init__(self, data=None):
        self.vehicles_states: Dict[str, ApolloVehicleState] = dict()
        self._frame = 0
        self.traffic_lights: Dict[str, TrafficLightColor] = dict()
        if data is not None:
            self.from_dict(data)

    @property
    def frame(self) -> int:
        return self._frame

    def set_frame(self, frame):
        self._frame = frame

    @classmethod
    def from_dict(cls, data):
        model = ApolloWorldModel()
        model._frame = data['frame']
        for uid, v_data in data['vehicle_state'].items():
            model.vehicles_states[uid] = ApolloVehicleState.from_dict(v_data)
        model.traffic_lights = dict()
        for uid, color_txt in data['traffic_light'].items():
            model.traffic_lights[uid] = COLOR_MAP[color_txt]
        return model

    def to_dict(self) -> dict:
        data = dict()

        data['frame'] = self.frame
        data['vehicle_state'] = {
            uid: self.get_vehicle_state(uid).to_dict() for uid in self.vehicle_ids
        }
        data['traffic_light'] = {
            uid: self.traffic_lights[uid].name for uid in self.traffic_lights
        }
        return data

    def copy(self):
        return ApolloWorldModel.from_dict(self.to_dict())

    def remove_vehicle(self, vehicle_id: str):
        self.vehicles_states.pop(vehicle_id)

    @property
    def autopilot_ids(self):
        return [uid for uid in self.vehicle_ids if uid.startswith('apollo')]

    @property
    def vehicle_ids(self):
        return self.vehicles_states.keys()

    def get_vehicle_state(self, uid: str):
        return self.vehicles_states[uid]

    def set_vehicle_state(self, uid: str, state: ApolloVehicleState):
        self.vehicles_states[uid] = state

    def set_traffic_light_state(self, uid: str, state: TrafficLightColor):
        self.traffic_lights[uid] = state

    def clear_traffic_light(self):
        self.traffic_lights = dict()
