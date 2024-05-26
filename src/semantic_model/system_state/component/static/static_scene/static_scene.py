from src.common.types import *
from src.semantic_model.system_state.component.static.metric_graph import MetricGraph
from src.semantic_model.system_state.component.static.static_object import StaticObject
from src.semantic_model.map.common import StaticObjectType
from src.semantic_model.system_state.proto import static_scene_pb2


class StaticScene(ProtoClass):
    @singledispatchmethod
    def __init__(self, metric_graph: MetricGraph, static_objects: Iterable[StaticObject]):
        self._metric_graph = metric_graph
        self._static_objects: Mapping[str, StaticObject] = MappingProxyType({
            static_object.id: static_object for static_object in static_objects
        })

        _node_static_objects_map = defaultdict(list)
        for static_object in self._static_objects.values():
            for node_id in static_object.state.control_node_ids:
                _node_static_objects_map[node_id].append(static_object)
        self._node_static_objects_map: Mapping[str, Tuple[StaticObject, ...]] = MappingProxyType({
            node_id: tuple(static_objects) for node_id, static_objects in _node_static_objects_map.items()
        })

    @__init__.register
    def __init__proto(self, proto: static_scene_pb2.StaticScene):
        self.__init__(MetricGraph(proto.metric_graph),
                      [StaticObject(static_object_proto) for static_object_proto in proto.static_objects])

    def dump(self) -> static_scene_pb2.StaticScene:
        proto = static_scene_pb2.StaticScene()
        proto.metric_graph.CopyFrom(self.metric_graph.dump())
        for static_object in self.static_objects.values():
            static_object_proto = proto.static_objects.add()
            static_object_proto.CopyFrom(static_object.dump())
        return proto

    @property
    def metric_graph(self) -> MetricGraph:
        return self._metric_graph

    @property
    def static_objects(self) -> Mapping[str, StaticObject]:
        return self._static_objects

    def static_object(self, object_id: str) -> StaticObject:
        return self.static_objects[object_id]

    def node_static_objects(self, node_id: str) -> Tuple[StaticObject, ...]:
        return self._node_static_objects_map[node_id] if node_id in self._node_static_objects_map else tuple()

    @cached_property
    def stop_signs(self) -> Mapping[str, StaticObject]:
        return MappingProxyType({
            static_object.id: static_object for static_object in self.static_objects.values()
            if static_object.type == StaticObjectType.STATIC_OBJECT_TYPE_STOP_SIGN
        })

    @cached_property
    def traffic_lights(self) -> Mapping[str, StaticObject]:
        return MappingProxyType({
            static_object.id: static_object for static_object in self.static_objects.values()
            if static_object.type == StaticObjectType.STATIC_OBJECT_TYPE_TRAFFIC_LIGHT
        })


def StaticSceneFromFile(map_name: str):
    with open(f'data/map/{map_name}/static_scene.bin', 'rb') as f:
        static_scene_proto = static_scene_pb2.StaticScene()
        static_scene_proto.ParseFromString(f.read())
        static_scene = StaticScene(static_scene_proto)