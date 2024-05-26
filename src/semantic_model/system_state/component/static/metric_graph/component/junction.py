from src.common.types import *
from src.semantic_model.system_state.proto import metric_graph_pb2


class Junction(ProtoClass):
    @singledispatchmethod
    def __init__(self, id_: str, edge_ids: Iterable[str],
                 entrance_node_ids: Iterable[str], exit_node_ids: Iterable[str]):
        self._id = id_
        self._edge_ids = frozenset(edge_ids)
        self._entrance_node_ids = tuple(entrance_node_ids)
        self._exit_node_ids = tuple(exit_node_ids)

    @__init__.register
    def __init__proto(self, proto: metric_graph_pb2.Junction):
        self.__init__(proto.id, [edge_id for edge_id in proto.edge_ids],
                      [node_id for node_id in proto.entrance_node_ids],
                      [node_id for node_id in proto.exit_node_ids])

    def dump(self) -> 'metric_graph_pb2.Junction':
        proto = metric_graph_pb2.Junction()
        proto.id = self.id
        for edge_id in self.edge_ids:
            proto.edge_ids.append(edge_id)
        for node_id in self.entrance_node_ids:
            proto.entrance_node_ids.append(node_id)
        for node_id in self.exit_node_ids:
            proto.exit_node_ids.append(node_id)
        return proto

    @property
    def id(self):
        return self._id

    @property
    def edge_ids(self) -> FrozenSet[str]:
        return self._edge_ids

    @property
    def entrance_node_ids(self) -> Tuple[str, ...]:
        return self._entrance_node_ids

    @property
    def entrance_num(self) -> int:
        return len(self.entrance_node_ids)

    @property
    def exit_node_ids(self) -> Tuple[str, ...]:
        return self._exit_node_ids

    @property
    def exit_num(self) -> int:
        return len(self.exit_node_ids)

