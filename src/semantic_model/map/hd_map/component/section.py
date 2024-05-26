from src.common.types import *
from src.semantic_model.map.common import RJType, TurnType
from src.semantic_model.geometry import Polyline
from src.semantic_model.map.hd_map.proto import hd_map_pb2
from .lane import Lane


class Section(ProtoClass):
    @singledispatchmethod
    def __init__(self, id_: str, reference_line: 'Polyline', ordered_lanes: Iterable['Lane'],
                 left_section_id: Optional[str], right_section_id: Optional[str]):
        self._id = id_
        self._ordered_lanes = tuple(ordered_lanes)
        self._lane_order: Mapping[str, int] = MappingProxyType({
            lane.id: order for order, lane in enumerate(self.ordered_lanes)
        })
        sample_lane = self.ordered_lanes[0]
        self._rj_type = sample_lane.rj_type
        self._turn_type = sample_lane.turn_type
        self._reference_line = reference_line
        self._left_section_id = left_section_id
        self._right_section_id = right_section_id

    @__init__.register
    def __init__proto(self, proto: hd_map_pb2.Section):
        self.__init__(proto.id, Polyline(proto.reference_line), [Lane(lane_proto) for lane_proto in proto.ordered_lanes],
                      proto.left_section_id if proto.HasField('left_section_id') else None,
                      proto.right_section_id if proto.HasField('right_section_id') else None)

    def dump(self) -> hd_map_pb2.Section:
        proto = hd_map_pb2.Section()
        proto.id = self.id
        for lane in self.ordered_lanes:
            lane_proto = proto.ordered_lanes.add()
            lane_proto.CopyFrom(lane.dump())
        proto.reference_line.CopyFrom(self.reference_line.dump())
        if self.left_section_id is not None:
            proto.left_section_id = self.left_section_id
        if self.right_section_id is not None:
            proto.right_section_id = self.right_section_id
        return proto

    @property
    def ordered_lanes(self) -> Tuple[Lane, ...]:
        return self._ordered_lanes

    def get_lane_order(self, lane_id: str) -> int:
        return self._lane_order[lane_id]

    @property
    def id(self) -> str:
        return self._id

    @property
    def reference_line(self) -> Polyline:
        return self._reference_line

    @property
    def length(self):
        return self.reference_line.length

    @property
    def turn_type(self) -> TurnType:
        return self._turn_type

    @property
    def rj_type(self) -> RJType:
        return self._rj_type

    @property
    def left_section_id(self) -> str:
        return self._left_section_id

    @property
    def right_section_id(self) -> str:
        return self._right_section_id

    def __eq__(self, other: 'Section') -> bool:
        return self.id == other.id

    def __str__(self):
        return f'Section({self.id})'

    def __repr__(self):
        return f'Section({self.id})'

    def __hash__(self):
        return hash(self.id)
