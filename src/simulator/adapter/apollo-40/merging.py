from src.simulator.apollo import ApolloSimulatorServer, ApolloSimulator, ApolloVehicleState, ApolloWorldModel, CarlaConfiguration
from src.simulator.apollo.config import APOLLO_SIMULATION_CONFIG
import math
import os
import json
from src.simulator.adapter.argument import make_arg_parser
from src.simulator.context import Context
import loguru

Context.vl_kmh = 40
Context.vl_mps = 40 / 3.6

args = make_arg_parser('apollo', 'merging').parse_args()

EGO_ID = 'apollo-1'
ARRIVING_ID = 'apollo-2'
FRONT_ID = 'static-10'

# D_MIN = APOLLO_SIMULATION_CONFIG.vehicle_shape.length + 6  # 5.4
D_MIN = 6
DC = 24
SHAPE = APOLLO_SIMULATION_CONFIG.apollo_vehicle_shape

MERGING_POINT_Y = -996 

from src.common.libs.apollo.map.parser import ApolloMapParser
from src.simulator.adapter.vista_util.apollo_hd_map_transformer import ApolloHDMapTransformer


def apollo_map_to_hd_map(apollo_map_string):
    parser = ApolloMapParser(apollo_map_string)
    hd_map = ApolloHDMapTransformer(parser).transform()
    return hd_map


with open('map_data/apollo/merging-40.txt') as f:
    apollo_map_data = f.read()

with open('map_data/carla/merging.xodr') as f:
    opendrive_map_data = f.read()


hd_map = apollo_map_to_hd_map(apollo_map_data)


ego_lane = None

for lane in hd_map.lanes.values():
    if lane.length > 1000:
        ego_lane = lane

assert ego_lane is not None


initial_world_state = ApolloWorldModel()
destinations = dict()


initial_world_state.set_vehicle_state(
    FRONT_ID,
    ApolloVehicleState(
        x=1.75, 
        y=MERGING_POINT_Y - SHAPE.back_edge_to_center - D_MIN - args.xf,
        v=0, a=0, theta_radians=-math.pi / 2, kappa=0, shape=SHAPE
    )
)

ego_xy = ego_lane.get_xy_by_sl(ego_lane.length - args.xe, 0)
ego_segment = theta_radians=ego_lane.reference_line.get_line_segment_by_s(ego_lane.length - args.xe)
assert ego_segment is not None
assert ego_xy is not None


ego_xy = ego_xy + ego_segment.vec.unit * 19


initial_world_state.set_vehicle_state(
    EGO_ID,
    ApolloVehicleState(
        x=ego_xy.x, y=ego_xy.y, v=args.ve, a=0, theta_radians=ego_segment.vec.angle.r, kappa=0,
        shape=SHAPE)
)

destinations[EGO_ID] = (
    1.75,
    MERGING_POINT_Y - 500,
)

initial_world_state.set_vehicle_state(
    ARRIVING_ID,
    ApolloVehicleState(
        x=1.75,
        y=MERGING_POINT_Y + SHAPE.front_edge_to_center + args.xa,
        v=Context.vl_mps / 2, a=0, theta_radians=-math.pi / 2, kappa=0, shape=SHAPE)
)

destinations[ARRIVING_ID] = (
    1.75,
    MERGING_POINT_Y - 500,
)

result = 'unknown'
state_sequence = []


def finish():
    trace = {
        'xe': args.xe,
        've': args.ve,
        'xf': args.xf,
        'xa': args.xa,
        'state_sequence': state_sequence,
        'collision': 'yes' if result == 'collision' else 'no',
        'planning_failed': 'yes' if result == 'planning_failed' else 'no'
    }
    print(json.dumps(trace))
    os._exit(0)

static_tick = 0

def callback(simulator: ApolloSimulator, has_error: str):
    global ego_trace, arriving_trace, static_tick, result
    ego_state = simulator.world_state.get_vehicle_state(EGO_ID)
    arriving_state = simulator.world_state.get_vehicle_state(ARRIVING_ID)

    state_sequence.append(simulator.world_state.to_dict())
    if ego_state.v < 1e-3 and arriving_state.v < 2e-2:
        static_tick += 1
    else:
        static_tick = 0
    
    if static_tick > 20:
        finish()

    if has_error and 'collision' in has_error.lower():
        result = 'collision'
        finish()

    if has_error and 'failed' in has_error.lower():
        result = 'planning_failed'
        finish()

    if simulator.collision:
        result = 'collision'
        finish()

carla_config = CarlaConfiguration('127.0.0.1', 2000, opendrive_map_data)

other_config = {
    'carla_offset_x': 0,
    'carla_offset_y': 3.5
}

server = ApolloSimulatorServer('bridge_server', 12345, initial_world_state,
                                     destinations, apollo_map_data, carla_config,
                                     0.1, callback, other_config=other_config)

server.run()
