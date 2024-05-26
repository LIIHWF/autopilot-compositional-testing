from src.semantic_model.map.hd_map import HDMap, LaneBoundaryType, Lane, Section, HDStaticObject, HDControlPosition
from src.semantic_model.map.common import TurnType, RJType, StaticObjectType
from src.common.types import *
from src.semantic_model.geometry import *
from functools import cmp_to_key
from src.common.libs.apollo.map.parser import ApolloMapParser
import os, argparse


BoundaryTypeMap = {
    0: LaneBoundaryType.LANE_BOUNDARY_TYPE_VIRTUAL,
    2: LaneBoundaryType.LANE_BOUNDARY_TYPE_DOTTED_WHITE,
    1: LaneBoundaryType.LANE_BOUNDARY_TYPE_DOTTED_YELLOW,
    5: LaneBoundaryType.LANE_BOUNDARY_TYPE_DOUBLE_YELLOW,
    4: LaneBoundaryType.LANE_BOUNDARY_TYPE_SOLID_WHITE,
    3: LaneBoundaryType.LANE_BOUNDARY_TYPE_SOLID_YELLOW,
    6: LaneBoundaryType.LANE_BOUNDARY_TYPE_CURB
}

TurnTypeMap = {
    4: TurnType.TURN_TYPE_U_TURN,
    1: TurnType.TURN_TYPE_NO_TURN,
    2: TurnType.TURN_TYPE_LEFT_TURN,
    3: TurnType.TURN_TYPE_RIGHT_TURN
}


class LaneCluster:
    def __init__(self, lanes: List[Lane]):
        self._lanes: List[Lane] = lanes
        self._lane_section_map: Dict[str, int] = {lane.id: sid for sid, lane in enumerate(self._lanes)}
        self._section_lanes_map: Dict[int, FrozenSet[Lane]] = {sid: frozenset({lane}) for sid, lane in enumerate(self._lanes)}
        self._process_cluster()
        self.clusters = frozenset(self._section_lanes_map.values())

    def _process_cluster(self):
        flag = True
        while flag:
            flag = False

            for i in range(len(self._lanes)):
                for j in range(i + 1, len(self._lanes)):
                    lane1 = self._lanes[i]
                    lane2 = self._lanes[j]
                    if self._lane_section_map[lane1.id] == self._lane_section_map[lane2.id]:
                        continue

                    if lane2.id == lane1.left_forward_neighbor_id:
                        self._union_section(lane1.id, lane2.id)
                        flag = True
                    elif lane2.id == lane1.right_forward_neighbor_id:
                        self._union_section(lane1.id, lane2.id)
                        flag = True
                    elif self._has_same_vertices(lane1.id, lane2.id) and \
                            lane1.rj_type == lane2.rj_type == RJType.RJ_TYPE_JUNCTION and lane1.turn_type == lane2.turn_type:
                        self._union_section(lane1.id, lane2.id)
                        flag = True

    def _union_section(self, lane1_id: str, lane2_id: str):
        section1_id = self._lane_section_map[lane1_id]
        section2_id = self._lane_section_map[lane2_id]
        for lane in self._section_lanes_map[section2_id]:
            self._lane_section_map[lane.id] = section1_id
        self._section_lanes_map[section1_id] |= self._section_lanes_map[section2_id]
        self._section_lanes_map.pop(section2_id)

    def _has_same_vertices(self, lane1_id: str, lane2_id: str) -> bool:
        section1_id = self._lane_section_map[lane1_id]
        section2_id = self._lane_section_map[lane2_id]
        return self._get_section_predecessors_id(section1_id) == self._get_section_predecessors_id(section2_id) and \
               self._get_section_successors_id(section1_id) == self._get_section_successors_id(section2_id)

    def _get_section_predecessors_id(self, sec_id: int) -> Set[int]:
        ret = set()
        for lane in self._section_lanes_map[sec_id]:
            for pred_lane_id in lane.predecessor_ids:
                pred_section_id = self._lane_section_map[pred_lane_id]
                ret.add(pred_section_id)
        return ret

    def _get_section_successors_id(self, sec_id: int) -> Set[int]:
        ret = set()
        for lane in self._section_lanes_map[sec_id]:
            for suc_lane_id in lane.successor_ids:
                suc_section = self._lane_section_map[suc_lane_id]
                ret.add(suc_section)
        return ret


def get_junction_lane_ids(map_proto) -> Set[str]:
    junction_lane_ids = set()
    overlap_map = {overlap.id.id: overlap for overlap in map_proto.overlap}
    for junction in map_proto.junction:
        for overlap_id in junction.overlap_id:
            if 'lane' in overlap_id.id:
                overlap = overlap_map[overlap_id.id]
                for obj in overlap.object:
                    if 'lane' in obj.id.id:
                        junction_lane_ids.add(obj.id.id)
    return junction_lane_ids


def get_lane_list_from_proto(proto_lanes, junction_lane_ids: Set[str]) -> List[Lane]:
    lanes = []
    for lane_proto in proto_lanes:
        lane_id = lane_proto.id.id
        lanes.append(Lane(lane_id,
                          [pred.id for pred in lane_proto.predecessor_id],
                          [suc.id for suc in lane_proto.successor_id],
                          lane_proto.left_neighbor_forward_lane_id[0].id
                          if len(lane_proto.left_neighbor_forward_lane_id) > 0 else None,
                          lane_proto.right_neighbor_forward_lane_id[0].id
                          if len(lane_proto.right_neighbor_forward_lane_id) > 0 else None,
                          lane_proto.left_neighbor_reverse_lane_id[0].id
                          if len(lane_proto.left_neighbor_reverse_lane_id) > 0 else None,
                          lane_proto.right_neighbor_reverse_lane_id[0].id
                          if len(lane_proto.right_neighbor_reverse_lane_id) > 0 else None,
                          BoundaryTypeMap[lane_proto.left_boundary.boundary_type[0].types[0]],
                          BoundaryTypeMap[lane_proto.right_boundary.boundary_type[0].types[0]],
                          RJType.RJ_TYPE_JUNCTION if lane_proto.id.id in junction_lane_ids else RJType.RJ_TYPE_ROAD,
                          TurnTypeMap[lane_proto.turn],
                          lane_proto.speed_limit,
                          Polyline(*[Vertex(point.x, point.y) for point in
                                     lane_proto.central_curve.segment[0].line_segment.point]),
                          Polyline(*[Vertex(point.x, point.y) for point in
                                     lane_proto.left_boundary.curve.segment[0].line_segment.point]),
                          Polyline(*[Vertex(point.x, point.y) for point in
                                     lane_proto.right_boundary.curve.segment[0].line_segment.point])
                          ))
    return lanes


def is_left_lane(lane1: 'Lane', lane2: 'Lane') -> bool:
    if lane1 == lane2:
        return False
    return lane1.reference_line.line_segments[0].inner_to_left_test(lane2.reference_line.vertices[0])


def get_ordered_lanes(lanes: Iterable[Lane]) -> List[Lane]:
    ordered_lanes = list(lanes)
    ordered_lanes.sort(key=cmp_to_key(
        lambda lane1, lane2: 0 if lane1.id == lane2.id else 1 if is_left_lane(lane1, lane2) else -1
    ))
    return ordered_lanes


def get_section_lanes_map(lane_list: List[Lane], section_id_prefix) -> Mapping[str, List[Lane]]:
    section_lanes_map: Dict[str, List[Lane]] = dict()
    section_counter = count()
    for lanes in LaneCluster(lane_list).clusters:
        section_id = f'{section_id_prefix}{next(section_counter)}'
        section_lanes_map[section_id] = get_ordered_lanes(lanes)
    return MappingProxyType(section_lanes_map)


def get_section_left_right_neighbors(section_lanes_map: Mapping[str, List[Lane]]):
    section_left_neighbor_id_map: Dict[str, str] = dict()
    section_right_neighbor_id_map: Dict[str, str] = dict()

    for section1_id, section1_lanes in section_lanes_map.items():
        for section2_id, section2_lanes in section_lanes_map.items():
            if section1_id != section2_id:
                if section1_lanes[0].left_reverse_neighbor_id == section2_lanes[0].id or \
                        section2_lanes[0].left_reverse_neighbor_id == section1_lanes[0].id:
                    section_left_neighbor_id_map[section1_id] = section2_id
                    section_left_neighbor_id_map[section2_id] = section1_id
                if section1_lanes[-1].right_reverse_neighbor_id == section2_lanes[-1].id or \
                        section2_lanes[-1].right_reverse_neighbor_id == section1_lanes[-1].id:
                    section_right_neighbor_id_map[section1_id] = section2_id
                    section_right_neighbor_id_map[section2_id] = section1_id
    return section_left_neighbor_id_map, section_right_neighbor_id_map


def generate_sections_by_lanes(lanes: List[Lane], section_id_prefix: str) -> List[Section]:
    section_lanes_map = get_section_lanes_map(lanes, section_id_prefix)
    section_left_neighbor_id_map, section_right_neighbor_id_map = \
        get_section_left_right_neighbors(section_lanes_map)
    sections = []
    for section_id, section_lanes in section_lanes_map.items():
        sections.append(Section(
            section_id, central_line(section_lanes[0].reference_line, section_lanes[-1].reference_line), section_lanes,
            section_left_neighbor_id_map[section_id] if section_id in section_left_neighbor_id_map else None,
            section_right_neighbor_id_map[section_id] if section_id in section_right_neighbor_id_map else None
        ))
    return sections


def get_static_object(static_object_proto, static_object_type: StaticObjectType, overlaps, lane_ids) -> HDStaticObject:
    control_positions = []
    for overlap_id in static_object_proto.overlap_id:
        overlap = overlaps[overlap_id.id]
        for obj_proto in overlap.object:
            if obj_proto.id.id in lane_ids:
                lane_id = obj_proto.id.id
                offset = obj_proto.lane_overlap_info.start_s
                control_positions.append(HDControlPosition(lane_id, offset))
    return HDStaticObject(static_object_proto.id.id, static_object_type, tuple(control_positions))


def get_static_objects(map_proto, lane_ids) -> List[HDStaticObject]:
    overlaps = {
        overlap_proto.id.id: overlap_proto for overlap_proto in map_proto.overlap
    }
    static_objects = []
    for traffic_light_proto in map_proto.signal:
        static_objects.append(get_static_object(traffic_light_proto,
                                                StaticObjectType.STATIC_OBJECT_TYPE_TRAFFIC_LIGHT,
                                                overlaps, lane_ids))
    for stop_sign_proto in map_proto.stop_sign:
        static_objects.append(get_static_object(stop_sign_proto,
                                                StaticObjectType.STATIC_OBJECT_TYPE_STOP_SIGN,
                                                overlaps, lane_ids))
    for yield_sign_proto in getattr(map_proto, 'yield'):
        static_objects.append(get_static_object(yield_sign_proto, StaticObjectType.STATIC_OBJECT_TYPE_YIELD_SIGN, overlaps, lane_ids))
    return static_objects


class ApolloHDMapTransformer:
    def __init__(self, apollo_map: ApolloMapParser, section_id_prefix: str = 'section_'):
        self._apollo_map = apollo_map
        self._section_id_prefix = section_id_prefix

    def transform(self) -> HDMap:
        junction_lane_ids = get_junction_lane_ids(self._apollo_map.proto)
        lanes = get_lane_list_from_proto(self._apollo_map.proto.lane, junction_lane_ids)
        lane_ids = set(lane.id for lane in lanes)
        sections = generate_sections_by_lanes(lanes, self._section_id_prefix)
        static_objects = get_static_objects(self._apollo_map.proto, lane_ids)
        return HDMap(sections, static_objects)


if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser('Apollo HD Map Transformer')
    arg_parser.add_argument('MAP_NAME', help='map name')
    arg_parser.add_argument('OUTPUT_DIR', help='absolute path of output folder')

    args = arg_parser.parse_args()

    MAP_NAME = args.MAP_NAME
    OUTPUT_DIR = args.OUTPUT_DIR

    with open(os.path.join('data/apollo_map', f'{MAP_NAME}.bin'), 'rb') as apollo_map_file:
        apollo_map_parser = ApolloMapParser(apollo_map_file.read())

    HD_MAP_DIR = os.path.join(OUTPUT_DIR, MAP_NAME)
    os.makedirs(HD_MAP_DIR, exist_ok=True)

    hd_map_proto = ApolloHDMapTransformer(apollo_map_parser).transform().dump()
    with open(os.path.join(HD_MAP_DIR, 'hd_map.bin'), 'wb') as hd_map_file:
        hd_map_file.write(hd_map_proto.SerializeToString())
    with open(os.path.join(HD_MAP_DIR, 'hd_map.txt'), 'w') as hd_map_file:
        hd_map_file.write(str(hd_map_proto))
