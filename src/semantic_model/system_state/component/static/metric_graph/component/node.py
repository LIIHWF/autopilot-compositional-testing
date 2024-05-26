from src.common.types import *
from src.semantic_model.geometry import Vertex
from src.semantic_model.system_state.proto import metric_graph_pb2


class Node(ProtoClass):
    @singledispatchmethod
    def __init__(self, node_id: str, position: Vertex):
        self._id = node_id
        self._position = position

    @__init__.register
    def __init__proto(self, proto: metric_graph_pb2.Node):
        self.__init__(proto.id, Vertex(proto.position))

    def dump(self) -> 'metric_graph_pb2.Node':
        proto = metric_graph_pb2.Node()
        proto.id = self.id
        if self.position is not None:
            proto.position.CopyFrom(self.position.dump())
        return proto

    @property
    def position(self):
        return self._position

    @property
    def id(self) -> str:
        return self._id

    def __eq__(self, other: 'Node'):
        return self.id == other.id

    def __repr__(self):
        return f'Node({self.id})'

    def __str__(self):
        return f'Node({self.id})'

    def __hash__(self):
        return hash(self.id)
