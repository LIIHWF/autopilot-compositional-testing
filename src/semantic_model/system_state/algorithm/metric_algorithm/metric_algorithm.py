from common.types import *
from semantic_model.geometry import *
from semantic_model.system_state import VehicleItinerary, VehicleItineraryBase, MetricGraph, StaticScene, \
    StaticObject, Position, VehicleState
from semantic_model.map.common import StaticObjectType, TurnType, RJType
from queue import PriorityQueue


END_CONFLICT_TURN_PRIORITY = {
    TurnType.TURN_TYPE_RIGHT_TURN: 4,
    TurnType.TURN_TYPE_NO_TURN: 3,
    TurnType.TURN_TYPE_LEFT_TURN: 2,
    TurnType.TURN_TYPE_U_TURN: 1
}


class FacingStaticObjectInfo:
    def __init__(self, static_object: StaticObject, node_id: str, distance: Number):
        self._static_object = static_object
        self._node_id = node_id
        self._distance = distance

    @property
    def static_object(self) -> StaticObject:
        return self._static_object

    @property
    def node_id(self) -> str:
        return self._node_id

    @property
    def distance(self) -> Number:
        return self._distance


class FacingConflictPointInfo:
    def __init__(self, ego_edge_id: str, ego_edge_offset: Number,
                 lateral_edge_id: str, lateral_edge_offset: Number,
                 ego_distance: Number):
        self._ego_edge_id = ego_edge_id
        self._ego_edge_offset = ego_edge_offset
        self._lateral_edge_id = lateral_edge_id
        self._lateral_edge_offset = lateral_edge_offset
        self._ego_distance = ego_distance

    @property
    def ego_edge_id(self) -> str:
        return self._ego_edge_id

    @property
    def ego_edge_offset(self) -> Number:
        return self._ego_edge_offset

    @property
    def lateral_edge_id(self) -> str:
        return self._lateral_edge_id

    @property
    def lateral_edge_offset(self) -> Number:
        return self._lateral_edge_offset

    @property
    def ego_distance(self) -> Number:
        return self._ego_distance


class MetricAlgorithm:
    def __init__(self, static_scene: StaticScene):
        self._static_scene = static_scene
        self._metric_graph = static_scene.metric_graph

    @property
    def static_scene(self) -> StaticScene:
        return self._static_scene

    @property
    def metric_graph(self) -> MetricGraph:
        return self._metric_graph

    def p2p_min_dist(self, start_position: Position, end_position: Position) -> Optional[Number]:
        distance = 0
        mg = self.static_scene.metric_graph
        if start_position.is_edge:
            start_edge = mg.edge(start_position.edge_id)
            start_node = mg.node(start_edge.target_id)
            distance += start_edge.length - start_position.offset
        else:
            start_node = mg.node(start_position.node_id)

        if end_position.is_edge:
            end_edge = mg.edge(end_position.edge_id)
            end_node = mg.node(end_edge.source_id)
            distance += end_position.offset
        else:
            end_node = mg.node(end_position.node_id)
        if start_node.id == end_node.id:
            return distance

        min_dist_map = dict()
        min_dist_map[start_node.id] = 0
        visited = set()
        visited.add(start_node.id)
        q = PriorityQueue()
        q.put((0, start_node.id))
        while not q.empty():
            cur_dist, cur_id = q.get()
            if cur_id in visited:
                if cur_id == end_node.id:
                    return distance + cur_dist
                continue
            for out_edge in mg.out_edges(cur_id):
                if out_edge.target_id not in min_dist_map:
                    min_dist_map[out_edge.target_id] = cur_dist + out_edge.length
                elif min_dist_map[out_edge.target_id] > cur_dist + out_edge.length:
                    min_dist_map[out_edge.target_id] = cur_dist + out_edge.length
                    q.put((cur_dist + out_edge.length, out_edge.target_id))

        if end_node in min_dist_map:
            return distance + min_dist_map[end_node.id]
        return INF

    def is_intersection(self, junction_id: str) -> bool:
        junction = self.metric_graph.junction(junction_id)
        if len(junction.edge_ids) == 2:
            return False
        for edge1_id in junction.edge_ids:
            for edge2_id in junction.edge_ids:
                if edge1_id != edge2_id:
                    if self.is_crossing(edge1_id, edge2_id):
                        return True
        return False

    def is_merger(self, junction_id: str) -> bool:
        junction = self.metric_graph.junction(junction_id)
        if len(junction.edge_ids) != 2:
            return False
        edge1_id, edge2_id = tuple(junction.edge_ids)
        edge1, edge2 = self.metric_graph.edge(edge1_id), self.metric_graph.edge(edge2_id)
        if edge1.target_id == edge2.target_id:
            return True
        return False

    def is_crossing(self, edge1_id: str, edge2_id: str) -> bool:
        assert edge1_id != edge2_id
        edge1 = self.metric_graph.edge(edge1_id)
        edge2 = self.metric_graph.edge(edge2_id)
        if edge1.target_id == edge2.source_id or edge2.target_id == edge1.source_id:
            return False
        if edge1.source_id == edge2.source_id or edge1.target_id == edge2.target_id:
            return True
        return edge1.reference_line.intersect_with(edge2.reference_line)

    def is_conflict(self, edge1_id: str, edge2_id: str) -> bool:
        assert edge1_id != edge2_id
        edge1 = self.metric_graph.edge(edge1_id)
        edge2 = self.metric_graph.edge(edge2_id)
        return self.is_crossing(edge1_id, edge2_id) and edge1.source_id != edge2.source_id

    def is_middle_conflict(self, edge1_id: str, edge2_id: str) -> bool:
        assert edge1_id != edge2_id
        edge1 = self.metric_graph.edge(edge1_id)
        edge2 = self.metric_graph.edge(edge2_id)
        return self.is_conflict(edge1_id, edge2_id) and edge1.target_id != edge2.target_id

    def is_end_conflict(self, edge1_id: str, edge2_id: str) -> bool:
        assert edge1_id != edge2_id
        edge1 = self.metric_graph.edge(edge1_id)
        edge2 = self.metric_graph.edge(edge2_id)
        return edge1.target_id == edge2.target_id

    def position_align_to_left(self, edge_id: str, offset: Number) -> Optional[Tuple[str, Number]]:
        edge = self.metric_graph.edge(edge_id)
        left_edge = self.metric_graph.left_adjacent_edge(edge_id)
        if left_edge is None:
            return None
        return left_edge.id, offset / edge.length * left_edge.length

    def position_align_to_right(self, edge_id: str, offset: Number) -> Optional[Tuple[str, Number]]:
        edge = self.metric_graph.edge(edge_id)
        right_edge = self.metric_graph.right_adjacent_edge(edge_id)
        if right_edge is None:
            return None
        return right_edge.id, offset / edge.length * right_edge.length

    def edges_crossing_distance(self, edge1_id, edge2_id) -> Tuple[Number, Number]:
        if not self.is_crossing(edge1_id, edge2_id):
            raise ValueError
        edge1 = self.metric_graph.edge(edge1_id)
        edge2 = self.metric_graph.edge(edge2_id)
        if edge1.source_id == edge2.source_id:
            return 0, 0
        if edge1.target_id == edge2.target_id:
            return edge1.length, edge2.length
        else:
            crossing_point = edge1.reference_line.intersect_point(edge2.reference_line)
            if crossing_point is None:
                raise RuntimeError(f'{edge1_id} {edge2_id}')
            distance1 = edge1.reference_line.get_sl_by_xy(crossing_point.x, crossing_point.y).s
            distance2 = edge2.reference_line.get_sl_by_xy(crossing_point.x, crossing_point.y).s
            return distance1, distance2

    def is_right_of(self, node1_id: str, node2_id: str) -> bool:
        for edge1 in self.metric_graph.out_edges(node1_id):
            for edge2 in self.metric_graph.out_edges(node2_id):
                if edge1.target_id != edge2.target_id:
                    continue
                if any([
                    edge1.turn_type == TurnType.TURN_TYPE_RIGHT_TURN and edge2.turn_type == TurnType.TURN_TYPE_NO_TURN,
                    edge1.turn_type == TurnType.TURN_TYPE_NO_TURN and edge2.turn_type == TurnType.TURN_TYPE_LEFT_TURN]):
                    return True
        return False

    def is_opposite(self, node1_id: str, node2_id: str) -> bool:
        junction1 = self.metric_graph.node_junction(node1_id)
        junction2 = self.metric_graph.node_junction(node2_id)
        if junction1 is None or junction2 is None or junction1.id != junction2.id:
            return False
        junction = junction1
        if node1_id not in junction.entrance_node_ids or node2_id not in junction.entrance_node_ids:
            return False
        for edge1 in self.metric_graph.out_edges(node1_id):
            for edge2 in self.metric_graph.out_edges(node2_id):
                if edge1.turn_type == edge2.turn_type == TurnType.TURN_TYPE_NO_TURN:
                    if edge1.target_id in self.metric_graph.junction_neighboring_exit_node_ids(node2_id) \
                            or edge2.target_id in self.metric_graph.junction_neighboring_exit_node_ids(node1_id):
                        return True
        return False

    def has_static_priority(self, edge1_id: str, edge2_id: str) -> bool:
        if not self.is_conflict(edge1_id, edge2_id):
            raise ValueError
        edge1 = self.metric_graph.edge(edge1_id)
        edge2 = self.metric_graph.edge(edge2_id)
        if all([
            len(self.static_scene.node_static_objects(edge2.source_id)) > 0,
            any((static_object.type == StaticObjectType.STATIC_OBJECT_TYPE_STOP_SIGN)
                for static_object in self.static_scene.node_static_objects(edge2.source_id)),
            len(self.static_scene.node_static_objects(edge1.source_id)) == 0
        ]):
            return True
        if all([
            len(self.static_scene.node_static_objects(edge1.source_id)) > 0,
            any((static_object.type == StaticObjectType.STATIC_OBJECT_TYPE_STOP_SIGN)
                for static_object in self.static_scene.node_static_objects(edge1.source_id)),
            len(self.static_scene.node_static_objects(edge2.source_id)) == 0
        ]):
            return False
        if self.is_right_of(edge1.source_id, edge2.source_id):
            return True
        if self.is_right_of(edge2.source_id, edge1.source_id):
            return False
        else:
            if END_CONFLICT_TURN_PRIORITY[edge1.turn_type] != END_CONFLICT_TURN_PRIORITY[edge2.turn_type]:
                return END_CONFLICT_TURN_PRIORITY[edge1.turn_type] > END_CONFLICT_TURN_PRIORITY[edge2.turn_type]
            else:
                offset1, offset2 = self.edges_crossing_distance(edge1.id, edge2.id)
                return offset1 < offset2

    def facing_static_object_info(self, itinerary: VehicleItinerary, skip: int = 0) -> Optional[FacingStaticObjectInfo]:
        distance = -itinerary.edge_offset
        for edge_id in itinerary.edge_id_sequence[:-1]:
            edge = self.metric_graph.edge(edge_id)
            distance += edge.length
            static_objects = self.static_scene.node_static_objects(edge.target_id)
            if len(static_objects) == 0:
                continue
            if len(static_objects) == 1:
                if skip <= 0:
                    return FacingStaticObjectInfo(static_objects[0], edge.target_id, distance)
                skip -= 1
            else:
                raise NotImplementedError
        return None

    def facing_edge_target_distance(self, itinerary: VehicleItinerary) -> Number:
        edge = self.metric_graph.edge(itinerary.current_edge_id)
        return edge.length - itinerary.edge_offset

    def all_conflict_point_info_within_distance(self, itinerary: VehicleItinerary, distance_limit: Number) -> List[FacingConflictPointInfo]:
        conflict_point_info_list = []
        cum_distance = -itinerary.edge_offset
        for index, edge_id in enumerate(itinerary.edge_id_sequence):
            edge = self.metric_graph.edge(edge_id)
            if edge.rj_type == RJType.RJ_TYPE_JUNCTION:
                junction = self.metric_graph.edge_junction(edge_id)
                for j_edge_id in junction.edge_ids:
                    j_edge = self.metric_graph.edge(j_edge_id)
                    if j_edge_id == edge_id:
                        continue
                    if self.is_conflict(edge.id, j_edge.id) and not self.has_static_priority(edge.id, j_edge.id):
                        ego_edge_offset, lateral_edge_offset = self.edges_crossing_distance(edge.id, j_edge.id)
                        if index == 0 and (ego_edge_offset < itinerary.edge_offset or
                                           ego_edge_offset - itinerary.edge_offset > distance_limit):
                            continue
                        if index == len(itinerary.edge_id_sequence) - 1 and ego_edge_offset > itinerary.end_offset:
                            continue
                        if cum_distance + ego_edge_offset > distance_limit:
                            continue
                        conflict_point_info_list.append(FacingConflictPointInfo(edge_id, ego_edge_offset,
                                                                                j_edge_id, lateral_edge_offset,
                                                                                cum_distance + ego_edge_offset))
            cum_distance += edge.length
        return conflict_point_info_list

    def facing_conflict_point_info(self, itinerary: VehicleItinerary, at_least_distance: Number = 0) -> Optional[FacingConflictPointInfo]:
        cum_distance = -itinerary.edge_offset
        for index, edge_id in enumerate(itinerary.edge_id_sequence):
            edge = self.metric_graph.edge(edge_id)
            if edge.rj_type == RJType.RJ_TYPE_JUNCTION:
                junction = self.metric_graph.edge_junction(edge_id)
                min_offset = INF
                min_j_edge_id = None
                min_j_edge_distance = None
                for j_edge_id in junction.edge_ids:
                    j_edge = self.metric_graph.edge(j_edge_id)
                    if j_edge_id == edge_id:
                        continue
                    if self.is_conflict(edge.id, j_edge.id) and not self.has_static_priority(edge.id, j_edge.id):
                        ego_edge_offset, lateral_edge_offset = self.edges_crossing_distance(edge.id, j_edge.id)
                        if index == 0 and (ego_edge_offset < itinerary.edge_offset or
                                           ego_edge_offset - itinerary.edge_offset < at_least_distance):
                            continue
                        if index == len(itinerary.edge_id_sequence) - 1 and ego_edge_offset > itinerary.end_offset:
                            continue
                        if cum_distance + ego_edge_offset < at_least_distance:
                            continue
                        if ego_edge_offset < min_offset:
                            min_offset = ego_edge_offset
                            min_j_edge_id = j_edge_id
                            min_j_edge_distance = lateral_edge_offset
                if min_offset != INF:
                    return FacingConflictPointInfo(edge_id, min_offset, min_j_edge_id,
                                                   min_j_edge_distance, cum_distance + min_offset)
            cum_distance += edge.length
        return None

    def itinerary_reference_line(self, itinerary: Union[VehicleItinerary, VehicleItineraryBase]) -> Polyline:
        if isinstance(itinerary, VehicleItineraryBase):
            itinerary = VehicleItinerary(itinerary, 0, itinerary.start_offset)
        metric_graph = self.metric_graph
        start_edge = metric_graph.edge(itinerary.current_edge_id)
        end_edge = metric_graph.edge(itinerary.edge_id_sequence[-1])
        start_point = start_edge.reference_line.get_xy_by_sl(itinerary.edge_offset, 0)
        start_line_segment_order = start_edge.reference_line.get_line_segment_order_by_s(itinerary.edge_offset)
        end_line_segment_order = end_edge.reference_line.get_line_segment_order_by_s(itinerary.end_offset)
        vertices = [start_point]
        for line_segment in start_edge.reference_line.line_segments[start_line_segment_order:
        end_line_segment_order if start_edge == end_edge else -1]:
            vertices.append(line_segment.v2)
        for edge_id in itinerary.edge_id_sequence[1: -1]:
            edge = metric_graph.edge(edge_id)
            vertices.append(metric_graph.node(edge.source_id).position)
            for line_segment in edge.reference_line.line_segments[:-1]:
                vertices.append(line_segment.v2)

        if start_edge.id != end_edge.id:
            vertices.append(metric_graph.node(end_edge.source_id).position)
            for line_segment in end_edge.reference_line.line_segments[:end_line_segment_order]:
                vertices.append(line_segment.v2)
        end_point = end_edge.reference_line.get_xy_by_sl(itinerary.end_offset, 0)
        vertices.append(end_point)
        return Polyline(*vertices)

    def traveled_distance(self, ego_state: VehicleState):
        if ego_state.itinerary.itinerary_index == 0:
            return ego_state.itinerary.edge_offset - ego_state.itinerary_base.start_offset
        cum_distance = -ego_state.itinerary_base.start_offset
        for i in range(ego_state.itinerary.itinerary_index):
            cum_distance += self.static_scene.metric_graph.edge(ego_state.itinerary_base.edge_id_sequence[i]).length
        cum_distance += ego_state.itinerary.edge_offset
        return cum_distance

    def crossing_point_distance(self,
                                itinerary1: Union[VehicleItinerary, VehicleItineraryBase],
                                itinerary2: Union[VehicleItinerary, VehicleItineraryBase]) \
            -> Optional[Tuple[Tuple[Number, Number], Tuple[Tuple[str, Number], Tuple[str, Number]]]]:
        metric_graph = self.metric_graph
        if len(set(itinerary1.edge_id_sequence) & set(itinerary2.edge_id_sequence)) > 0:
            return None
        reference_line1 = self.itinerary_reference_line(itinerary1)
        reference_line2 = self.itinerary_reference_line(itinerary2)
        xy_point = reference_line1.intersect_point(reference_line2)
        if xy_point is None:
            return None
        distance1 = reference_line1.get_sl_by_xy(xy_point.x, xy_point.y).s
        distance2 = reference_line2.get_sl_by_xy(xy_point.x, xy_point.y).s

        for edge1_id in itinerary1.edge_id_sequence:
            for edge2_id in itinerary2.edge_id_sequence:
                edge1 = metric_graph.edge(edge1_id)
                edge2 = metric_graph.edge(edge2_id)
                if edge1.reference_line.intersect_with(edge2.reference_line):
                    intersect_edge1_id = edge1.id
                    intersect_edge2_id = edge2.id
                    offset1 = edge1.reference_line.get_sl_by_xy(xy_point.x, xy_point.y).s
                    offset2 = edge2.reference_line.get_sl_by_xy(xy_point.x, xy_point.y).s
                    return (distance1, distance2), ((intersect_edge1_id, offset1), (intersect_edge2_id, offset2))
        return None

    def front_position(self, itinerary: VehicleItinerary, distance: Number) -> Optional[Tuple[str, Number]]:
        if distance > self.itinerary_reference_line(itinerary).length:
            return None
        start_edge = self.metric_graph.edge(itinerary.current_edge_id)
        if distance <= start_edge.length - itinerary.edge_offset:
            return start_edge.id, itinerary.start_offset + distance
        if itinerary.itinerary_index + 1 >= len(itinerary.itinerary_base.edge_id_sequence):
            return None
        return self.front_position(VehicleItinerary(itinerary.itinerary_base, itinerary.itinerary_index + 1, 0),
                                   distance - (start_edge.length - itinerary.start_offset))

    def back_position(self, edge_id: str, offset: Number, distance: Number,
                      back_edge_ids: Optional[Tuple[str, ...]] = None) -> Optional[Tuple[str, Number]]:
        if offset >= distance:
            return edge_id, offset - distance
        else:
            pre_edge_ids = [e.id for e in self.metric_graph.in_edges(
                self.metric_graph.node(self.metric_graph.edge(edge_id).source_id).id)]
            k = 0
            if len(pre_edge_ids) != 1:
                while k < len(back_edge_ids):
                    if back_edge_ids[k] in pre_edge_ids:
                        pre_edge_id = back_edge_ids[k]
                        k += 1
                        break
                    k += 1
                else:
                    return None
            else:
                pre_edge_id = list(pre_edge_ids)[0]
            return self.back_position(pre_edge_id, self.metric_graph.edge(pre_edge_id).length, distance - offset,
                                      None if back_edge_ids is None or k >= len(back_edge_ids) else back_edge_ids[k:])

    def front_distance_p2p(self, edge_1_id: str, offset_1: Number, edge_2_id: str, offset_2: Number) -> Optional[Number]:
        if edge_1_id == edge_2_id and offset_2 >= offset_1:
            return offset_2 - offset_1
        edge_1 = self.metric_graph.edge(edge_1_id)
        length = -edge_1.length
        visited_id = set()
        while True:
            length += edge_1.length
            if edge_1.id in visited_id:
                return None
            visited_id.add(edge_1.id)
            if edge_1.id == edge_2_id:
                length += offset_2
                return length
            out_edges = self.metric_graph.out_edges(edge_1.target_id)
            if len(out_edges) != 1:
                return None
            edge_1 = next(iter(out_edges))

    def front_distance_by_edge_offset(self, itinerary: VehicleItinerary, edge_id: str, offset: Number)\
            -> Optional[Number]:
        if edge_id not in itinerary.edge_id_sequence:
            return None
        if edge_id == itinerary.edge_id_sequence[0] and offset < itinerary.start_offset:
            return None
        if edge_id == itinerary.edge_id_sequence[-1] and offset > itinerary.end_offset:
            return None

        if edge_id == itinerary.edge_id_sequence[0]:
            return offset - itinerary.start_offset

        distance = -itinerary.edge_offset
        for eid in itinerary.edge_id_sequence:
            if eid == edge_id:
                return distance + offset
            distance += self.metric_graph.edge(eid).length
        raise RuntimeError

    def front_distance_by_node_id(self, itinerary: VehicleItinerary, node_id: str) -> Optional[Number]:
        distance = 0
        for index, edge_id in enumerate(itinerary.edge_id_sequence):
            edge = self.metric_graph.edge(edge_id)
            if index == 0:
                distance += edge.length - itinerary.edge_offset
            else:
                distance += edge.length
            if edge.target_id == node_id:
                return distance
        return None
