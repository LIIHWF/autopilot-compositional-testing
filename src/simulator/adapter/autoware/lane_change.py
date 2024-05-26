from src.simulator.autoware import *
import math
import sys, os, json
import argparse
import loguru

from src.simulator.adapter.argument import make_arg_parser
from src.simulator.context import Context

os.system('script/switch_autoware_map.sh lane_change')
args = make_arg_parser('autoware', 'lane_change').parse_args()


world_model = AutowareWorldModel()
destinations = dict()

shape = AutowareVehicleShape(3.889, 0.977, 1.0455, 1.0455, 1.441)


xff = args.xff
ve = args.ve
xa = args.xa
xf = args.xf
side_offset = 13.6

start_offset = 500


EGO_ID = 'v1'
ARRIVING_ID = 'v2'
FRONT_ID = 'front'

world_model.set_vehicle_state('v1', AutowareVehicleState(
    1.75, start_offset - 0.3 * ve, ve, 0, math.pi / 2,
    shape
), VehicleMode.AUTOPILOT)
if xf is not None:
    destinations['v1'] = {
        'x': -1.75, 'y': start_offset + side_offset + xf, 'theta': math.pi / 2
    }
    # destinations['v1'] = {
    #     'x': 1.75, 'y': 900, 'theta': math.pi / 2
    # }
else:
    destinations['v1'] = {
        'x': -1.75, 'y': 900, 'theta': math.pi / 2
    }

if xa is not None:
    world_model.set_vehicle_state('v2', AutowareVehicleState(
        -1.75, start_offset + side_offset - xa -
        shape.length, 80 / 3.6, 0, math.pi / 2,
        shape
    ), VehicleMode.AUTOPILOT)
    destinations['v2'] = {
        'x': -1.75, 'y': 900, 'theta': math.pi / 2
    }

world_model.set_vehicle_state('obstacle', AutowareVehicleState(
    1.75, start_offset + 5 + xff + shape.length, 0, 0, math.pi / 2,
    shape
), VehicleMode.STATIC)

front_y = start_offset + side_offset + xf + 5
world_model.set_vehicle_state('front', AutowareVehicleState(
    -1.75, front_y, 0, 0, math.pi / 2,
    shape
), VehicleMode.STATIC)

ego_trace = []
arriving_trace = []
front_state = None
collision = False


def finish():
    global collision
    assert front_state is not None
    sys.stdout = sys.__stdout__
    print(json.dumps({
        'xf\'': xff,
        've': ve,
        'xa': xa,
        'xf': xf,
        'ego_trace': ego_trace,
        'arriving_trace': arriving_trace,
        'front_state': front_state.to_dict(),
        'collision': 'yes' if collision else 'no'
    }))
    os._exit(0)

with open('map_data/carla/lane_change.xodr', 'r') as f:
    map_data = f.read()

config = {
    'carla_offset_x': 0,
    'carla_offset_y': 0,
    'carla_flip': True
}

end_cnt = 0

def callback(simulator: AutowareSimulator, error_message):
    global front_state, collision, end_cnt
    ego_state = simulator.autoware_world_state.get_vehicle_state(EGO_ID)
    arriving_state = simulator.autoware_world_state.get_vehicle_state(
        ARRIVING_ID)
    obstacle_state = simulator.autoware_world_state.get_vehicle_state('obstacle')
    ego_trace.append(ego_state.to_dict())
    arriving_trace.append(arriving_state.to_dict())
    front_state = simulator.autoware_world_state.get_vehicle_state(FRONT_ID)

    if simulator.collision:
        collision = True
        loguru.logger.error('Collision detected')
        finish()
        
    if ego_state.v < 1e-5 and arriving_state.v < 1e-5:
        loguru.logger.success('Finish for all vehicle stop')
        finish()

    if ego_state.v < 1e-5 and (arriving_state.y > start_offset + 5):
        end_cnt += 1
    else:
        end_cnt = 0
    if end_cnt > 10:
        loguru.logger.success('Finish for ego stop')
        finish()

    if simulator.frame > 600:
        loguru.logger.error('Timeout')
        finish()

server = AutowareSimulatorServer('server', 12346, world_model,
                                 destinations, map_data, ['v1'],
                                 enable_carla=True, config=config, callback=callback)

server.run()
