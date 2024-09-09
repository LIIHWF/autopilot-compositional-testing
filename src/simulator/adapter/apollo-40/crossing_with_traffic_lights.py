from src.simulator.apollo import ApolloSimulatorServer, ApolloSimulator, ApolloVehicleState, ApolloWorldModel, CarlaConfiguration
from src.simulator.apollo.config import APOLLO_SIMULATION_CONFIG
import math
import os
import json
from src.simulator.adapter.argument import make_arg_parser
from src.simulator.context import Context

Context.vl_kmh = 40
Context.vl_mps = 40 / 3.6

args = make_arg_parser('apollo', 'crossing_with_traffic_lights').parse_args()

EGO_ID = 'apollo-1'
FRONT_ID = 'static-10'

D_MIN = 6
DC = 24
SHAPE = APOLLO_SIMULATION_CONFIG.apollo_vehicle_shape

initial_world_state = ApolloWorldModel()
destinations = dict()

initial_world_state.set_vehicle_state(EGO_ID,
                                      ApolloVehicleState(
                                          x=1.75, y=-12 - SHAPE.front_edge_to_center - args.xe - 0.2 * args.ve - 0.01, v=args.ve, a=0, theta_radians=math.pi/2, kappa=0,
                                          shape=SHAPE
                                      ))
destinations[EGO_ID] = (1.75, 900)

initial_world_state.set_vehicle_state(FRONT_ID,
                                      ApolloVehicleState(
                                          x=1.75, y=12 + SHAPE.back_edge_to_center + args.xf + D_MIN + 0.1, v=0, a=0, theta_radians=math.pi/2, kappa=0,
                                          shape=SHAPE
                                      ))


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
        'xe': args.xe,
        've': args.ve,
        'xf': args.xf,
        'state_sequence': state_sequence,
        'collision': 'yes' if result == 'collision' else 'no'
    }
    print(json.dumps(trace))
    os._exit(0)


trace = []

result = 'unknown'
safety_issues = []

static_cnt = 0

def callback(simulator: ApolloSimulator, has_error: str):
    global result, static_cnt
    trace.append(simulator.world_state.to_dict())
    ego_state = simulator.world_state.get_vehicle_state(EGO_ID)

    state_sequence.append(simulator.world_state.to_dict())

    if simulator.frame > 150 and ego_state.y < 0:
        finish()

    if ego_state.v < 1e-3:
        static_cnt += 1
    else:
        static_cnt = 0
    
    if static_cnt > 50:
        finish()

    if has_error and 'collision' in has_error.lower():
        result = 'collision'
        finish()

    if simulator.collision:
        result = 'collision'
        finish()

with open('map_data/apollo/crossing_with_traffic_lights-40.txt') as f:
    apollo_map_data = f.read()

with open('map_data/carla/crossing_with_traffic_lights.xodr') as f:
    opendrive_map_data = f.read()

carla_config = CarlaConfiguration('127.0.0.1', 2000, opendrive_map_data)

server = ApolloSimulatorServer('bridge_server', 12345, initial_world_state,
                                     destinations, apollo_map_data, carla_config,
                                     0.1, callback, has_traffic_light=True)

server.run()
