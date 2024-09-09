from src.simulator.apollo import ApolloSimulatorServer, ApolloSimulator, ApolloVehicleState, ApolloWorldModel, CarlaConfiguration
from src.simulator.apollo.config import APOLLO_SIMULATION_CONFIG
import math
import os
import json
import argparse
from src.simulator.adapter.argument import make_arg_parser
from src.simulator.context import Context

Context.vl_kmh = 40
Context.vl_mps = 40 / 3.6

args = make_arg_parser('apollo', 'lane_change').parse_args()

EGO_ID = 'apollo-1'
ARRIVING_ID = 'apollo-2'
OBSTACLE_ID = 'static-9'
FRONT_ID = 'static-10'

D_MIN = 6
DC = 24
SHAPE = APOLLO_SIMULATION_CONFIG.apollo_vehicle_shape

THETA = math.pi / 12
OFFSET = 3.5 / math.tan(THETA)


initial_world_state = ApolloWorldModel()
destinations = dict()

initial_world_state.set_vehicle_state(
    OBSTACLE_ID,
    ApolloVehicleState(
        500 + args.xff + D_MIN + SHAPE.back_edge_to_center,
        y=-1.75, v=0, a=0, theta_radians=0, kappa=0, shape=SHAPE)
)


initial_world_state.set_vehicle_state(
    FRONT_ID,
    ApolloVehicleState(
        x=500 + OFFSET + args.xf + D_MIN + SHAPE.back_edge_to_center,
        y=1.75, v=0, a=0, theta_radians=0, kappa=0, shape=SHAPE
    )
)


initial_world_state.set_vehicle_state(
    EGO_ID,
    ApolloVehicleState(
        x=500 - SHAPE.front_edge_to_center, y=-1.75, v=args.ve, a=0, theta_radians=0, kappa=0,
        shape=SHAPE)
)
destinations[EGO_ID] = (
    500 + OFFSET + args.xf + SHAPE.length,
    1.75
)


initial_world_state.set_vehicle_state(
    ARRIVING_ID,
    ApolloVehicleState(
        x=500 + OFFSET - args.xa - SHAPE.back_edge_to_center,
        y=1.75, v=Context.vl_mps / 2, a=0, theta_radians=0, kappa=0, shape=SHAPE)
)
destinations[ARRIVING_ID] = (
    500 + OFFSET + args.xf + SHAPE.length,
    1.75
)

max_dist = -float('inf')

distance_list = []
speed_list = []
limit_list = []

ego_passed = False
arriving_passed = False
result = 'caution'
state_sequence = []


def finish():
    trace = {
        'xf\'': args.xff,
        've': args.ve,
        'xf': args.xf,
        'xa': args.xa,
        'state_sequence': state_sequence,
        'collision': 'yes' if result == 'collision' else 'no',
        'planning_failed': 'yes' if result == 'planning_failed' else 'no'
    }
    print(json.dumps(trace))
    os._exit(0)


trace = []

result = 'unknown'
safety_issues = []

static_cnt = 0

def callback(simulator: ApolloSimulator, has_error: str):
    global ego_passed, arriving_passed, result, static_cnt
    trace.append(simulator.world_state.to_dict())
    ego_state = simulator.world_state.get_vehicle_state(EGO_ID)
    arriving_state = simulator.world_state.get_vehicle_state(ARRIVING_ID)
    front_state = simulator.world_state.get_vehicle_state(FRONT_ID)

    state_sequence.append(simulator.world_state.to_dict())

    if (ego_state.v < 1e-6 and arriving_state.v < 1e-6):
        static_cnt += 1
    else:
        static_cnt = 0

    if simulator.frame > 500 or static_cnt > 50:
        finish()
    
    if (abs(ego_state.y - -1.75) < 0.5) and arriving_state.x > simulator.world_state.get_vehicle_state(OBSTACLE_ID).x + 10:
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


with open('map_data/apollo/lane_change-40.txt') as f:
    apollo_map_data = f.read()

with open('map_data/carla/lane_change.xodr') as f:
    opendrive_map_data = f.read()


carla_config = CarlaConfiguration('127.0.0.1', 2000, opendrive_map_data)

server = ApolloSimulatorServer('bridge_server', 12345, initial_world_state,
                                     destinations, apollo_map_data, carla_config,
                                     0.1, callback)

server.run()
