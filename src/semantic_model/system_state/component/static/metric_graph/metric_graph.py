from src.common.types import *
from .component.node import Node
from .component.edge import Edge
from .component.road import Road
from .component.junction import Junction
from src.semantic_model.system_state.proto import metric_graph_pb2


def get_enter_exit_edges(nodes, edges):
    enter_edges = {node_id: set() for node_id in nodes}
    exit_edges = {node_id: set() for node_id in nodes}
    for edge in edges.values():
        enter_edges[edge.target_id].add(edge)
        exit_edges[edge.source_id].add(edge)
    static_enter_edges = MappingProxyType({
        node_id: frozenset(edge_set) for node_id, edge_set in enter_edges.items()
    })
    static_exit_edges = MappingProxyType({
        node_id: frozenset(edge_set) for node_id, edge_set in exit_edges.items()
    })
    return static_enter_edges, static_exit_edges


class Position:
    def __init__(self, element_id: str, offset: Optional[Number] = None):
        self._element_id = element_id
        self._offset = offset

    @property
    def is_node(self) -> bool:
        return self._offset is None

    @property
    def is_edge(self) -> bool:
        return not self.is_node

    @property
    def offset(self) -> Number:
        if self.is_node:
            raise RuntimeError
        return self._offset

    @property
    def node_id(self) -> str:
        if not self.is_node:
            raise RuntimeError
        return self._element_id

    @property
    def edge_id(self) -> str:
        if not self.is_edge:
            raise RuntimeError
        return self._element_id


class MetricGraph(ProtoClass):
    @singledispatchmethod
    def __init__(self, nodes: Iterable[Node], edges: Iterable[Edge],
                 roads: Iterable[Road], junctions: Iterable[Junction],
                 junction_entex_vertex_relation: Iterable[Tuple[str, str]],
                 is_right_adjacent_edge_relation: Iterable[Tuple[str, str]]):
        self._nodes: Mapping[str, Node] = MappingProxyType({node.id: node for node in nodes})
        self._edges: Mapping[str, Edge] = MappingProxyType({edge.id: edge for edge in edges})
        self._roads: Mapping[str, Road] = MappingProxyType({road.id: road for road in roads})
        self._junctions: Mapping[str, Junction] = MappingProxyType({
            junction.id: junction for junction in junctions})
        self._in_edges, self._out_edges = get_enter_exit_edges(self.nodes, self.edges)
        self._edge_road_map: Mapping[str, Road] = MappingProxyType({
            edge_id: road for road in self.roads.values() for edge_id in road.edge_id_sequence})
        self._edge_junction_map: Mapping[str, Junction] = MappingProxyType({
            edge_id: junction for junction in self.junctions.values() for edge_id in junction.edge_ids})

        self._junction_neighboring_exit_node_ids_map: Mapping[str, List[str]] = defaultdict(list)
        self._junction_neighboring_entrance_node_ids_map: Mapping[str, List[str]] = defaultdict(list)
        for en_node_id, ex_node_id in junction_entex_vertex_relation:
            self._junction_neighboring_exit_node_ids_map[en_node_id].append(ex_node_id)
            self._junction_neighboring_entrance_node_ids_map[ex_node_id].append(en_node_id)

        self._right_adjacent_edge_id: Mapping[str, str] = MappingProxyType({
            edge2_id: edge1_id for edge1_id, edge2_id in is_right_adjacent_edge_relation
        })
        self._left_adjacent_edge_id: Mapping[str, str] = MappingProxyType({
            edge1_id: edge2_id for edge1_id, edge2_id in is_right_adjacent_edge_relation
        })

    @__init__.register
    def __init__proto(self, proto: metric_graph_pb2.MetricGraph):
        self.__init__(
            [Node(node_proto) for node_proto in proto.nodes],
            [Edge(edge_proto) for edge_proto in proto.edges],
            [Road(road_proto) for road_proto in proto.roads],
            [Junction(junction_proto) for junction_proto in proto.junctions],
            [(junction_entex_vertex_relation_proto.object1_id, junction_entex_vertex_relation_proto.object2_id)
             for junction_entex_vertex_relation_proto in proto.junction_entex_vertex_relation],
            [(is_right_adjacent_proto.object1_id, is_right_adjacent_proto.object2_id)
             for is_right_adjacent_proto in proto.is_right_adjacent_edge_relation]
        )

    def dump(self) -> 'metric_graph_pb2.MetricGraph':
        proto = metric_graph_pb2.MetricGraph()
        for node in self.nodes.values():
            node_proto = proto.nodes.add()
            node_proto.CopyFrom(node.dump())
        for edge in self.edges.values():
            edge_proto = proto.edges.add()
            edge_proto.CopyFrom(edge.dump())
        for road in self.roads.values():
            road_proto = proto.roads.add()
            road_proto.CopyFrom(road.dump())
        for junction in self.junctions.values():
            junction_proto = proto.junctions.add()
            junction_proto.CopyFrom(junction.dump())
        for entrance_node_id, exit_node_ids in self._junction_neighboring_exit_node_ids_map.items():
            for exit_node_id in exit_node_ids:
                entex_proto = proto.junction_entex_vertex_relation.add()
                entex_proto.object1_id = entrance_node_id
                entex_proto.object2_id = exit_node_id
        for is_right_adjacent_pair in self._left_adjacent_edge_id.items():
            is_right_adjacent_proto = proto.is_right_adjacent_edge_relation.add()
            is_right_adjacent_proto.object1_id = is_right_adjacent_pair[0]
            is_right_adjacent_proto.object2_id = is_right_adjacent_pair[1]
        return proto

    def junction_neighboring_exit_node_ids(self, entrance_node_id: str) -> Tuple[str, ...]:
        return tuple(self._junction_neighboring_entrance_node_ids_map[entrance_node_id]) \
            if entrance_node_id in self._junction_neighboring_exit_node_ids_map else tuple()

    def junction_neighboring_exit_nodes(self, entrance_node_id: str) -> Tuple[Node, ...]:
        nids = self.junction_neighboring_exit_node_ids(entrance_node_id)
        return tuple(self.node(nid) for nid in nids)

    def junction_neighboring_entering_node_ids(self, exit_node_id: str) -> Tuple[str, ...]:
        return tuple(self._junction_neighboring_entrance_node_ids_map[exit_node_id]) \
            if exit_node_id in self._junction_neighboring_entrance_node_ids_map else tuple

    def junction_neighboring_entrance_nodes(self, exit_node_id: str) -> Tuple[Node, ...]:
        nids = self.junction_neighboring_entering_node_ids(exit_node_id)
        return tuple(self.node(nid) for nid in nids)

    def right_adjacent_edge_id(self, edge_id: str) -> Optional[str]:
        return self._right_adjacent_edge_id[edge_id] if edge_id in self._right_adjacent_edge_id else None

    def right_adjacent_edge(self, edge_id: str) -> Optional[Edge]:
        right_adjacent_edge_id = self.right_adjacent_edge_id(edge_id)
        return self.edge(right_adjacent_edge_id) if right_adjacent_edge_id is not None else None

    def left_adjacent_edge_id(self, edge_id: str) -> Optional[str]:
        return self._left_adjacent_edge_id[edge_id] if edge_id in self._left_adjacent_edge_id else None

    def left_adjacent_edge(self, edge_id: str) -> Optional[Edge]:
        left_adjacent_edge_id = self.left_adjacent_edge_id(edge_id)
        return self.edge(left_adjacent_edge_id) if left_adjacent_edge_id is not None else None

    @property
    def nodes(self) -> Mapping[str, Node]:
        return self._nodes

    @property
    def edges(self) -> Mapping[str, Edge]:
        return self._edges

    def node(self, node_id: str) -> Node:
        return self.nodes[node_id]

    def edge(self, edge_id: str) -> Edge:
        return self.edges[edge_id]

    @property
    def roads(self) -> Mapping[str, Road]:
        return self._roads

    @property
    def junctions(self) -> Mapping[str, Junction]:
        return self._junctions

    def road(self, road_id: str) -> Road:
        return self.roads[road_id]

    def junction(self, junction_id: str) -> Junction:
        return self.junctions[junction_id]

    @property
    def edge_road_map(self) -> Mapping[str, Road]:
        return self._edge_road_map

    def edge_road(self, edge_id: str) -> Optional[Road]:
        return self._edge_road_map[edge_id] if edge_id in self._edge_road_map else None

    @property
    def edge_junction_map(self) -> Mapping[str, Junction]:
        return self._edge_junction_map

    def edge_junction(self, edge_id) -> Optional[Junction]:
        return self._edge_junction_map[edge_id] if edge_id in self._edge_junction_map else None

    def in_edges(self, node_id: str) -> FrozenSet[Edge]:
        return self._in_edges[node_id]

    def in_edge_ids(self, node_id: str) -> FrozenSet[str]:
        return frozenset(e.id for e in self.in_edges(node_id))

    def out_edges(self, node_id: str) -> FrozenSet[Edge]:
        return self._out_edges[node_id]

    def out_edge_ids(self, node_id: str) -> FrozenSet[str]:
        return frozenset(e.id for e in self.out_edges(node_id))

    def node_edges(self, node_id: str) -> FrozenSet[Edge]:
        return self.in_edges(node_id) | self.out_edges(node_id)

    def node_road(self, node_id: str) -> Optional[Road]:
        for edge in self.node_edges(node_id):
            road = self.edge_road(edge.id)
            if road is not None:
                return road
        return None

    def node_junction(self, node_id: str) -> Optional[Junction]:
        for edge in self.node_edges(node_id):
            junction = self.edge_junction(edge.id)
            if junction is not None:
                return junction
        return None

    def edge_source(self, edge_id: str) -> Node:
        return self.nodes[self.edges[edge_id].source_id]

    def edge_target(self, edge_id: str) -> Node:
        return self.nodes[self.edges[edge_id].target_id]

    def edge_length(self, edge_id: str) -> Number:
        return self.edges[edge_id].length

    def road_edge_sequence(self, road_id: str) -> Tuple[Edge, ...]:
        return tuple(self.edges[edge_id] for edge_id in self.roads[road_id].edge_id_sequence)

    def road_entrance(self, road_id: str) -> Node:
        return self.nodes[self.roads[road_id].entrance_id]

    def road_exit(self, road_id: str) -> Node:
        return self.nodes[self.roads[road_id].exit_id]

    def junction_edges(self, junction_id: str) -> FrozenSet[Edge]:
        return frozenset(self.edges[edge_id] for edge_id in self.junctions[junction_id].edge_ids)

    def junction_entrance_nodes(self, junction_id: str) -> Tuple[Node, ...]:
        return tuple(self.nodes[node_id] for node_id in self.junctions[junction_id].entrance_node_ids)

    def junction_entrance_num(self, junction_id: str) -> int:
        return self.junctions[junction_id].entrance_num

    def junction_exit_nodes(self, junction_id: str) -> Tuple[Node, ...]:
        return tuple(self.nodes[node_id] for node_id in self.junctions[junction_id].exit_node_ids)

    def junction_exit_num(self, junction_id: str) -> int:
        return self.junctions[junction_id].exit_num

    def sub_graph(self, node_ids: Iterable[str]):
        roads = set(self.node_road(node_id) for node_id in node_ids)
        junctions = set(self.node_junction(node_id) for node_id in node_ids)

        edge_ids = set()
        for road in roads:
            edge_ids |= set(road.edge_id_sequence)
        for junction in junctions:
            edge_ids |= junction.edge_ids
        edges = set(self.edge(edge_id) for edge_id in edge_ids)

        node_ids = set(edge.target_id for edge in edges) | set(edge.source_id for edge in edges)
        nodes = [self.node(node_id) for node_id in node_ids]

        junction_entex_vertex_relation = []
        for relation in self._junction_neighboring_exit_node_ids_map.items():
            node1_id = relation[0]
            if node1_id not in node_ids:
                continue
            for node2_id in relation[1]:
                if node2_id in node_ids:
                    junction_entex_vertex_relation.append((node1_id, node2_id))

        is_right_adjacent_edge_relation = []
        for relation in self._left_adjacent_edge_id.items():
            if (relation[0] in edge_ids) and (relation[1] in edge_ids):
                is_right_adjacent_edge_relation.append(relation)

        return MetricGraph(nodes, edges, roads, junctions,
                           junction_entex_vertex_relation, is_right_adjacent_edge_relation)
