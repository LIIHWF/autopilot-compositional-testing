from src.common.types import *
from src.semantic_model.map.common import RJType, TurnType
from src.semantic_model.system_state.proto import metric_graph_pb2
from src.semantic_model.geometry import Polyline


class Edge(ProtoClass):
    @singledispatchmethod
    def __init__(self, id_: str, source_id: str, target_id: str,
                 rj_type: RJType, turn_type: TurnType, speed_limit: Number, reference_line: Polyline):
        self._id = id_
        self._source_id = source_id
        self._target_id = target_id
        self._rj_type = rj_type
        self._turn_type = turn_type
        self._speed_limit = speed_limit
        self._reference_line = reference_line

    @__init__.register
    def __init__proto(self, proto: metric_graph_pb2.Edge):
        self.__init__(proto.id, proto.source_id, proto.target_id,
                      RJType(proto.rj_type), TurnType(proto.turn_type),
                      proto.speed_limit,
                      Polyline(proto.reference_line))

    def dump(self) -> 'metric_graph_pb2.Edge':
        proto = metric_graph_pb2.Edge()
        proto.id = self.id
        proto.source_id = self.source_id
        proto.target_id = self.target_id
        proto.rj_type = self.rj_type.value
        proto.turn_type = self.turn_type.value
        proto.speed_limit = self.speed_limit
        proto.reference_line.CopyFrom(self.reference_line.dump())
        return proto

    @property
    def id(self) -> str:
        return self._id

    @property
    def source_id(self) -> str:
        return self._source_id

    @property
    def target_id(self) -> str:
        return self._target_id

    @property
    def length(self) -> Number:
        return self.reference_line.length

    @property
    def rj_type(self) -> RJType:
        return self._rj_type

    def set_rj_type(self, rj_type: RJType):
        self._rj_type = rj_type

    @property
    def speed_limit(self) -> Number:
        return self._speed_limit

    @property
    def turn_type(self) -> TurnType:
        return self._turn_type

    @property
    def reference_line(self) -> Polyline:
        return self._reference_line

    def __eq__(self, other: 'Edge'):
        return self.id == other.id

    def __str__(self):
        return f'Edge({self.id}, {self.source_id}, {self.target_id}, {round(self.length, 3)})'

    def __hash__(self):
        return hash(self.id)

    def __repr__(self):
        return str(self)
