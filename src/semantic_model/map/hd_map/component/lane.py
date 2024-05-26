from src.common.types import *
from src.semantic_model.geometry import RegionPolyline, Polyline
from src.semantic_model.map.common import RJType, TurnType
from src.semantic_model.map.hd_map.proto import hd_map_pb2
from enum import Enum


LaneBoundaryType = Enum('LaneBoundaryType', {name: id_ for name, id_ in hd_map_pb2.LaneBoundaryType.items()})


class Lane(RegionPolyline):
    @singledispatchmethod
    def __init__(self, id_: str, predecessor_ids: Iterable[str], successor_ids: Iterable[str],
                 left_forward_neighbor_id: Optional[str], right_forward_neighbor_id: Optional[str],
                 left_reverse_neighbor_id: Optional[str], right_reverse_neighbor_id: Optional[str],
                 left_boundary_type: LaneBoundaryType, right_boundary_type: LaneBoundaryType,
                 rj_type_: RJType, turn_type: TurnType, speed_limit: Number,
                 reference_line: Polyline, left_boundary_line: Polyline, right_boundary_line: Polyline
                 ):
        self._id = id_
        self._left_forward_neighbor_id, self._right_forward_neighbor_id = \
            left_forward_neighbor_id, right_forward_neighbor_id
        self._left_reverse_neighbor_id, self._right_reverse_neighbor_id = \
            left_reverse_neighbor_id, right_reverse_neighbor_id
        self._left_boundary_type, self._right_boundary_type = \
            left_boundary_type, right_boundary_type
        self._predecessor_ids = frozenset(predecessor_ids)
        self._successor_ids = frozenset(successor_ids)
        self._rj_type = rj_type_
        self._turn_type = turn_type
        self._speed_limit = speed_limit
        super().__init__(reference_line, left_boundary_line, right_boundary_line)

    @__init__.register
    def __init__proto(self, proto: hd_map_pb2.Lane):
        self.__init__(
            proto.id, [pid for pid in proto.predecessor_ids], [sid for sid in proto.successor_ids],
            proto.left_forward_neighbor_id if proto.HasField('left_forward_neighbor_id') else None,
            proto.right_forward_neighbor_id if proto.HasField('right_forward_neighbor_id') else None,
            proto.left_reverse_neighbor_id if proto.HasField('left_reverse_neighbor_id') else None,
            proto.right_reverse_neighbor_id if proto.HasField('right_reverse_neighbor_id') else None,
            LaneBoundaryType(proto.left_boundary_type), LaneBoundaryType(proto.right_boundary_type),
            RJType(proto.rj_type), TurnType(proto.turn_type), proto.speed_limit,
            Polyline(proto.region_polyline.reference_line),
            Polyline(proto.region_polyline.left_boundary_line),
            Polyline(proto.region_polyline.right_boundary_line)
        )

    def dump(self) -> 'hd_map_pb2.Lane':
        proto = hd_map_pb2.Lane()
        proto.id = self.id
        proto.rj_type = self.rj_type.value
        for predecessor_id in self.predecessor_ids:
            proto.predecessor_ids.append(predecessor_id)
        for successor_id in self.successor_ids:
            proto.successor_ids.append(successor_id)
        if self.left_forward_neighbor_id is not None:
            proto.left_forward_neighbor_id = self.left_forward_neighbor_id
        if self.right_forward_neighbor_id is not None:
            proto.right_forward_neighbor_id = self.right_forward_neighbor_id
        if self.left_reverse_neighbor_id is not None:
            proto.left_reverse_neighbor_id = self.left_reverse_neighbor_id
        if self.right_reverse_neighbor_id is not None:
            proto.right_reverse_neighbor_id = self.right_reverse_neighbor_id
        proto.left_boundary_type = self.left_boundary_type.value
        proto.right_boundary_type = self.right_boundary_type.value
        proto.turn_type = self.turn_type.value
        proto.speed_limit = self.speed_limit
        proto.region_polyline.CopyFrom(super().dump())
        return proto

    @property
    def length(self):
        return self.reference_line.length

    @property
    def id(self) -> str:
        return self._id

    @property
    def rj_type(self) -> RJType:
        return self._rj_type

    @property
    def turn_type(self) -> TurnType:
        return self._turn_type

    @property
    def predecessor_ids(self) -> FrozenSet[str]:
        return self._predecessor_ids

    @property
    def successor_ids(self) -> FrozenSet[str]:
        return self._successor_ids

    @property
    def left_forward_neighbor_id(self) -> Optional[str]:
        return self._left_forward_neighbor_id

    @property
    def right_forward_neighbor_id(self) -> Optional[str]:
        return self._right_forward_neighbor_id

    @property
    def left_reverse_neighbor_id(self) -> Optional[str]:
        return self._left_reverse_neighbor_id

    @property
    def right_reverse_neighbor_id(self) -> Optional[str]:
        return self._right_reverse_neighbor_id

    @property
    def left_boundary_type(self) -> LaneBoundaryType:
        return self._left_boundary_type

    @property
    def right_boundary_type(self) -> LaneBoundaryType:
        return self._right_boundary_type

    @property
    def speed_limit(self) -> Number:
        return self._speed_limit

    def __eq__(self, other: 'Lane') -> bool:
        return self.id == other.id

    def __repr__(self):
        return f'Lane({self.id})'

    def __str__(self):
        return f'Lane({self.id})'

    def __hash__(self):
        return hash(self.id)
