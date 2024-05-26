from src.semantic_model.system_state.proto import dynamic_state_pb2
from src.common.types import *

from src.semantic_model.system_state.component.static.static_object import StaticObjectState

TrafficLightColor = Enum('TrafficLightColor',
                         {name: id_ for name, id_ in dynamic_state_pb2.TrafficLightColor.items()})


class TrafficLightDynamicState(ProtoClass):
    @singledispatchmethod
    def __init__(self, color: TrafficLightColor):
        self._color = color

    @__init__.register
    def __init__proto(self, proto: dynamic_state_pb2.TrafficLightDynamicState):
        self.__init__(TrafficLightColor(proto.color))

    def dump(self) -> dynamic_state_pb2.TrafficLightDynamicState:
        proto = dynamic_state_pb2.TrafficLightDynamicState()
        proto.color = self.color.value
        return proto

    @property
    def color(self):
        return self._color


class TrafficLightState:
    def __init__(self, static_state: StaticObjectState, dynamic_state: TrafficLightDynamicState):
        self._static_state = static_state
        self._dynamic_state = dynamic_state

    @property
    def static_state(self) -> StaticObjectState:
        return self._static_state

    @property
    def dynamic_state(self) -> TrafficLightDynamicState:
        return self._dynamic_state

    @property
    def color(self) -> TrafficLightColor:
        return self.dynamic_state.color

    @property
    def control_node_ids(self) -> Tuple[str, ...]:
        return self.static_state.control_node_ids
