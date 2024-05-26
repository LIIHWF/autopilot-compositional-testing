from common.types import *
from semantic_model.system_run import Snapshot
from semantic_model.system_state.algorithm.metric_algorithm import MetricAlgorithm, FacingStaticObjectInfo


def _init_edge_vehicle_ids_map(snapshot: Snapshot) -> Mapping[str, Tuple[str]]:
    edge_vehicle_id_map = defaultdict(list)
    for vehicle_id in snapshot.vehicle_ids:
        vehicle_state = snapshot.vehicle_state(vehicle_id)
        edge_vehicle_id_map[vehicle_state.edge_id].append(vehicle_id)
    return MappingProxyType({
        eid: vids for eid, vids in edge_vehicle_id_map.items()
    })


class DynamicAlgorithm:
    def __init__(self, snapshot: Snapshot):
        self._snapshot = snapshot
        self._metric_algorithm = MetricAlgorithm(snapshot.static_scene)
        self._at_node_vehicle_id_map: Optional[Mapping[str, str]] = None
        self._edge_vehicle_ids_map: Mapping[str, Tuple[str]] = _init_edge_vehicle_ids_map(snapshot)

    @property
    def snapshot(self) -> Snapshot:
        return self._snapshot

    @property
    def metric_algorithm(self) -> MetricAlgorithm:
        return self._metric_algorithm

    def _get_at_node_vehicle_id_map(self) -> Mapping[str, str]:
        at_node_vehicle_id_map = dict()
        for vehicle_id in self.snapshot.vehicle_ids:
            facing_info = self.at_static_object_info(vehicle_id)
            if facing_info is not None:
                at_node_vehicle_id_map[facing_info.node_id] = vehicle_id
        return MappingProxyType(at_node_vehicle_id_map)

    def at_node_vehicle_id(self, node_id: str) -> Optional[str]:
        if self._at_node_vehicle_id_map is None:
            self._at_node_vehicle_id_map = self._get_at_node_vehicle_id_map()
        if node_id not in self._at_node_vehicle_id_map:
            return None
        else:
            return self._at_node_vehicle_id_map[node_id]

    def at_static_object_info(self, vehicle_id: str) -> Optional[FacingStaticObjectInfo]:
        vehicle_state = self.snapshot.vehicle_state(vehicle_id)
        facing_info = self.metric_algorithm.facing_static_object_info(vehicle_state.itinerary)
        if facing_info is not None:
            if facing_info.distance <= 3:
                return facing_info

    def edge_vehicle_ids(self, edge_id: str) -> Tuple[str, ...]:
        return self._edge_vehicle_ids_map[edge_id] if edge_id in self._edge_vehicle_ids_map else tuple()
