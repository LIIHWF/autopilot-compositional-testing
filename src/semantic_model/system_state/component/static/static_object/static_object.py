from src.common.types import *
from src.semantic_model.system_state.proto import static_object_pb2
from src.semantic_model.map.common import StaticObjectType
from enum import Enum


class StaticObjectState(ProtoClass):
    @singledispatchmethod
    def __init__(self, control_node_ids: Tuple[str, ...]):
        self._control_node_ids = control_node_ids

    @__init__.register
    def __init__proto(self, proto: static_object_pb2.StaticObjectState):
        self.__init__(tuple(proto.control_node_ids))

    def dump(self) -> 'static_object_pb2.StaticObjectState':
        proto = static_object_pb2.StaticObjectState()
        for control_node_id in self.control_node_ids:
            proto.control_node_ids.append(control_node_id)
        return proto

    @property
    def control_node_ids(self):
        return self._control_node_ids

    def __eq__(self, other: 'StaticObjectState'):
        return self.control_node_ids == other.control_node_ids

    def __str__(self):
        return f'SignalState(control_node_id=\'{self.control_node_ids}\')'


class StaticObject(ProtoClass):
    @singledispatchmethod
    def __init__(self, id_: str, signal_type: StaticObjectType, state: StaticObjectState):
        self._id = id_
        self._signal_type = signal_type
        self._state = state

    @__init__.register
    def __init__proto(self, proto: static_object_pb2.StaticObject):
        self.__init__(proto.id, StaticObjectType(proto.type), StaticObjectState(proto.state))

    def dump(self) -> 'static_object_pb2.StaticObject':
        proto = static_object_pb2.StaticObject()
        proto.id = self.id
        proto.type = self.type.value
        proto.state.CopyFrom(self.state.dump())
        return proto

    @property
    def id(self) -> str:
        return self._id

    @property
    def type(self) -> StaticObjectType:
        return self._signal_type

    @property
    def state(self) -> StaticObjectState:
        return self._state

    def __eq__(self, other: 'StaticObject'):
        return self.id == other.id
