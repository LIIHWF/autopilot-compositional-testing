from src.simulator.autoware import *
import math
import matplotlib.pyplot as plt
import loguru
import json
import os, sys
import argparse
import time

from src.simulator.adapter.argument import make_arg_parser
from src.simulator.context import Context

os.system('script/switch_autoware_map.sh crossing_with_yield_signs')
args = make_arg_parser('autoware', 'crossing_with_yield_signs').parse_args()

world_model = AutowareWorldModel()
destinations = dict()

shape = AutowareVehicleShape(3.889, 0.977, 1.0455, 1.0455, 1.441)


xe = args.xe
ve = args.ve
xa = args.xa
xf = args.xf


EGO_ID = 'v1'
ARRIVING_ID = 'v2'
FRONT_ID = 'front'

world_model.set_vehicle_state(EGO_ID, AutowareVehicleState(
    1.75, -12 - xe - shape.front_edge_to_center, ve, 0, math.pi / 2,
    shape
), VehicleMode.AUTOPILOT)
destinations[EGO_ID] = {
    'x': 1.75, 'y': 800, 'theta': math.pi / 2
}

world_model.set_vehicle_state(ARRIVING_ID, AutowareVehicleState(
    - 12 - xa - shape.front_edge_to_center, -1.75, 80 / 3.6, 0, 0,
    shape
), VehicleMode.AUTOPILOT)
destinations[ARRIVING_ID] = {
    'x': 900, 'y': -1.75, 'theta': 0
}

world_model.set_vehicle_state(FRONT_ID, AutowareVehicleState(
    1.75, 12 + xf + 5 + shape.back_edge_to_center, 0, 0, math.pi / 2,
    shape
), VehicleMode.STATIC)

with open('map_data/carla/crossing_with_yield_signs.xodr') as f:
    map_data = f.read()

accs = []

ego_trace = []
arriving_trace = []
front_state = None


def finish():
    global collision
    assert front_state is not None
    print(json.dumps({
        'xe': xe,
        've': ve,
        'xa': xa,
        'xf': xf,
        'ego_trace': ego_trace,
        'arriving_trace': arriving_trace,
        'front_state': front_state.to_dict(),
        'collision': 'yes' if collision else 'no'
    }))
    os._exit(0)


collision = False

static_cnt = 0
def callback(simulator: AutowareSimulator, error_message: str):
    global front_state, collision, static_cnt
    ego_state = simulator.autoware_world_state.get_vehicle_state(EGO_ID)
    arriving_state = simulator.autoware_world_state.get_vehicle_state(
        ARRIVING_ID)
    ego_trace.append(ego_state.to_dict())
    arriving_trace.append(arriving_state.to_dict())
    front_state = simulator.autoware_world_state.get_vehicle_state(FRONT_ID)

    if ego_state.v < 1e-5 and front_state.y - ego_state.y < 15:
        static_cnt += 1
    else:
        static_cnt = 0
        
    if static_cnt > 50:
        finish()
        
    if simulator.collision:
        collision = True
        finish()

    if arriving_state.x > 100 and ego_state.v < 1e-5:
        finish()

    if simulator.frame > 600:
        finish()

server = AutowareSimulatorServer('server', 12346, world_model,
                                 destinations, map_data, callback=callback, 
                                 enable_carla=True, collision_detected_actor=['v1'])

server.run()
