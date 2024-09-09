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

args = make_arg_parser('apollo', 'crossing_with_yield_signs').parse_args()

EGO_ID = 'apollo-1'
ARRIVING_ID = 'apollo-2'
FRONT_ID = 'static-10'

D_MIN = 6
DC = 24
SHAPE = APOLLO_SIMULATION_CONFIG.apollo_vehicle_shape

initial_world_state = ApolloWorldModel()
destinations = dict()

initial_world_state.set_vehicle_state(EGO_ID,
                                      ApolloVehicleState(
                                          x=1.75, y=-12 - SHAPE.front_edge_to_center - args.xe - 0.1 * args.ve, v=args.ve, a=0, theta_radians=math.pi/2, kappa=0,
                                          shape=SHAPE
                                      ))
destinations[EGO_ID] = (1.75, 900)

initial_world_state.set_vehicle_state(ARRIVING_ID,
                                      ApolloVehicleState(
                                          x=-12 - SHAPE.front_edge_to_center - args.xa, y=-1.75, v=Context.vl_mps / 2, a=0, theta_radians=0, kappa=0,
                                          shape=SHAPE
                                      ))
destinations[ARRIVING_ID] = (900, -1.75)

initial_world_state.set_vehicle_state(FRONT_ID,
                                      ApolloVehicleState(
                                          x=1.75, y=12 + SHAPE.back_edge_to_center + args.xf + D_MIN, v=0, a=0, theta_radians=math.pi/2, kappa=0,
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
        'xa': args.xa,
        'state_sequence': state_sequence,
        'collision': 'yes' if result == 'collision' else 'no'
    }
    print(json.dumps(trace))
    os._exit(0)


trace = []

result = 'unknown'
safety_issues = []


def callback(simulator: ApolloSimulator, has_error: str):
    global ego_passed, arriving_passed, result
    trace.append(simulator.world_state.to_dict())
    ego_state = simulator.world_state.get_vehicle_state(EGO_ID)
    arriving_state = simulator.world_state.get_vehicle_state(ARRIVING_ID)
    front_state = simulator.world_state.get_vehicle_state(FRONT_ID)

    state_sequence.append(simulator.world_state.to_dict())

    # if (-12 < ego_state.front_position.y < 12):
    #     simulator.ignore_vehicles_map[ARRIVING_ID] = {FRONT_ID}
    # else:
    #     simulator.ignore_vehicles_map[ARRIVING_ID] = {FRONT_ID, EGO_ID}

    if simulator.frame > 500 or (front_state.y - ego_state.y < 15 and ego_state.v < 1e-6):
        finish()

    if result == 'unknown' and arriving_state.x > 1.75:
        result = 'caution'

    elif result == 'caution' and arriving_state.x > 2.5:
        result = 'progress'

    if has_error and 'collision' in has_error.lower():
        result = 'collision'
        finish()

    if simulator.collision:
        result = 'collision'
        finish()

with open('map_data/apollo/crossing_with_yield_signs-40.txt') as f:
    apollo_map_data = f.read()

with open('map_data/carla/crossing_with_yield_signs.xodr') as f:
    opendrive_map_data = f.read()

carla_config = CarlaConfiguration('127.0.0.1', 2000, opendrive_map_data)

server = ApolloSimulatorServer('bridge_server', 12345, initial_world_state,
                                     destinations, apollo_map_data, carla_config,
                                     0.1, callback)

server.run()
