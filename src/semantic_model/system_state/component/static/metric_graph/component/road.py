from src.common.types import *
from src.semantic_model.system_state.proto import metric_graph_pb2


class Road(ProtoClass):

    @singledispatchmethod
    def __init__(self, id_: str, edge_id_sequence: Iterable[str], entrance_id: str, exit_id: str):
        self._id = id_
        self._edge_id_sequence = tuple(edge_id_sequence)
        self._entrance_id: str = entrance_id
        self._exit_id: str = exit_id

    @__init__.register
    def __init__proto(self, proto: metric_graph_pb2.Road):
        self.__init__(proto.id, [edge_id for edge_id in proto.edge_id_sequence], proto.entrance_id, proto.exit_id)

    def dump(self) -> 'metric_graph_pb2.Road':
        proto = metric_graph_pb2.Road()
        proto.id = self.id
        for edge_id in self.edge_id_sequence:
            proto.edge_id_sequence.append(edge_id)
        proto.entrance_id = self.entrance_id
        proto.exit_id = self.exit_id
        return proto

    @property
    def id(self) -> str:
        return self._id

    @property
    def edge_id_sequence(self) -> Tuple[str, ...]:
        return self._edge_id_sequence

    @property
    def entrance_id(self) -> str:
        return self._entrance_id

    @property
    def exit_id(self) -> str:
        return self._exit_id

    def __eq__(self, other: 'Road'):
        return self.id == other.id

    def __hash__(self):
        return hash(self.id)
