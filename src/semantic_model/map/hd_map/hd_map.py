from src.common.types import *
from .component.lane import Lane
from .component.section import Section
from src.semantic_model.map.hd_map.proto import hd_map_pb2
from src.semantic_model.map.common import StaticObjectType


class HDControlPosition(ProtoClass):
    @singledispatchmethod
    def __init__(self, lane_id: str, offset: Number):
        self._lane_id = lane_id
        self._offset = offset

    @__init__.register
    def __init__proto(self, proto: hd_map_pb2.HdControlPosition):
        self.__init__(proto.lane_id, proto.offset)

    def dump(self) -> hd_map_pb2.HdControlPosition:
        proto = hd_map_pb2.HdControlPosition()
        proto.lane_id = self.lane_id
        proto.offset = self.offset
        return proto

    @property
    def lane_id(self) -> str:
        return self._lane_id

    @property
    def offset(self) -> Number:
        return self._offset


class HDStaticObject(ProtoClass):
    @singledispatchmethod
    def __init__(self, _id: str, _type: StaticObjectType, control_positions: Tuple[HDControlPosition, ...]):
        self._id = _id
        self._type = _type
        self._control_positions = control_positions

    @__init__.register
    def __init__proto(self, proto: hd_map_pb2.HdStaticObject):
        self.__init__(proto.id, StaticObjectType(proto.type),
                      tuple(HDControlPosition(cp_proto) for cp_proto in proto.control_positions))

    def dump(self) -> hd_map_pb2.HdStaticObject:
        proto = hd_map_pb2.HdStaticObject()
        proto.id = self.id
        proto.type = self.type.value
        for control_position in self.control_positions:
            cp_proto = proto.control_positions.add()
            cp_proto.CopyFrom(control_position.dump())
        return proto

    @property
    def id(self) -> str:
        return self._id

    @property
    def type(self) -> StaticObjectType:
        return self._type

    @property
    def control_positions(self) -> Tuple[HDControlPosition]:
        return self._control_positions


class HDMap(ProtoClass):
    @singledispatchmethod
    def __init__(self, sections: Iterable[Section], static_objects: Iterable[HDStaticObject]):
        self._sections: Mapping[str, Section] = MappingProxyType({
            section.id: section for section in sections
        })

        self._lanes: Mapping[str, Lane] = MappingProxyType({
            lane.id: lane for section in sections for lane in section.ordered_lanes
        })

        self._lane_section_map = MappingProxyType({
            lane.id: section for section in sections for lane in section.ordered_lanes
        })

        self._static_objects: Mapping[str, HDStaticObject] = MappingProxyType({
            static_object.id: static_object for static_object in static_objects
        })

    @__init__.register
    def __init__proto(self, proto: hd_map_pb2.HdMap):
        self.__init__([Section(section_proto) for section_proto in proto.sections],
                      [HDStaticObject(static_object_proto) for static_object_proto in proto.static_objects])

    def dump(self) -> hd_map_pb2.HdMap:
        proto = hd_map_pb2.HdMap()
        for section in self.sections.values():
            section_proto = proto.sections.add()
            section_proto.CopyFrom(section.dump())
        for static_object in self.static_objects.values():
            static_object_proto = proto.static_objects.add()
            static_object_proto.CopyFrom(static_object.dump())
        return proto

    @property
    def sections(self) -> Mapping[str, Section]:
        return self._sections

    def section(self, section_id: str) -> Section:
        return self.sections[section_id]

    @property
    def lanes(self) -> Mapping[str, Lane]:
        return self._lanes

    def lane(self, lane_id: str) -> Lane:
        return self.lanes[lane_id]

    def get_lane_section(self, lane_id: str) -> Section:
        return self._lane_section_map[lane_id]

    @property
    def static_objects(self) -> Mapping[str, HDStaticObject]:
        return self._static_objects

    def static_object(self, object_id: str) -> HDStaticObject:
        return self.static_objects[object_id]
