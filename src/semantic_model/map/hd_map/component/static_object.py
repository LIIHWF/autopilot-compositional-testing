from common.types import *
from semantic_model.map.common import RJType, TurnType, StaticObjectType
from semantic_model.geometry import Polyline
from semantic_model.map.hd_map.proto import hd_map_pb2
from .lane import Lane


class ControlPosition(ProtoClass):
    @singledispatchmethod
    def __init__(self, lane_id: str, offset: Number):
        self._lane_id = lane_id
        self._offset = offset

    @__init__.register
    def __init__proto(self, proto: hd_map_pb2.StaticObject.ControlPosition):
        self.__init__(proto.lane_id, proto.offset)

    def dump(self) -> hd_map_pb2.StaticObject.ControlPosition:
        proto = hd_map_pb2.StaticObject.ControlPosition()
        proto.lane_id = self.lane_id
        proto.offset = self.offset
        return proto

    @property
    def lane_id(self) -> str:
        return self._lane_id

    @property
    def offset(self) -> Number:
        return self._offset


class StaticObject(ProtoClass):
    @singledispatchmethod
    def __init__(self, _type: StaticObjectType, control_positions: Tuple[ControlPosition, ...]):
        self._type=  _type
        self._control_positions = control_positions

    @__init__.register
    def __init__proto(self, proto: hd_map_pb2.StaticObject):
        self.__init__(StaticObjectType(proto.type),
            tuple(ControlPosition(control_position_proto) for control_position_proto in proto.control_positions))

    def dump(self) -> hd_map_pb2.StaticObject:
        proto = hd_map_pb2.StaticObject()
        proto.type = self.type.value
        for control_position in self.control_positions:
            control_position_proto = proto.control_positions.add()
            control_position_proto.CopyFrom(control_position.dump())
        return proto

    @property
    def type(self) -> StaticObjectType:
        return self._type

    @property
    def control_positions(self) -> Tuple[ControlPosition, ...]:
        return self._control_positions
