from src.simulator.autoware import *
import math
import sys, os, json
import loguru
import argparse
import os
from src.common.libs.apollo.map.parser import ApolloMapParser
from src.simulator.adapter.vista_util.apollo_hd_map_transformer import ApolloHDMapTransformer
from src.simulator.adapter.argument import make_arg_parser
from src.simulator.context import Context

os.system('script/switch_autoware_map.sh merging')

args = make_arg_parser('autoware', 'merging').parse_args()

shape = AutowareVehicleShape(3.889, 0.977, .0455, 1.0455, 1.441)

world_model = AutowareWorldModel()
destinations = dict()

xe = args.xe
ve = args.ve
xa = args.xa
xf = args.xf


def apollo_map_to_hd_map(apollo_map_string):
    parser = ApolloMapParser(apollo_map_string)
    hd_map = ApolloHDMapTransformer(parser).transform()
    return hd_map


with open('map_data/apollo/merging.txt') as f:
    apollo_map_data = f.read()

with open('map_data/carla/merging.xodr') as f:
    opendrive_map_data = f.read()


hd_map = apollo_map_to_hd_map(apollo_map_data)


ego_lane = None

for lane in hd_map.lanes.values():
    if lane.length > 1000:
        ego_lane = lane

assert ego_lane is not None


lane = ego_lane = hd_map.lane('lane_2')

EGO_ID = 'v1'
ARRIVING_ID = 'v2'
FRONT_ID = 'front'


ego_position = lane.get_xy_by_sl(lane.length - xe, 0)

ego_xy = ego_lane.get_xy_by_sl(ego_lane.length - args.xe, 0)
ego_segment = theta_radians=ego_lane.reference_line.get_line_segment_by_s(ego_lane.length - args.xe)
assert ego_segment is not None
assert ego_xy is not None

assert ego_position is not None
ego_theta = lane.reference_line.get_line_segment_by_s(
    lane.length - xe).unit.angle

ego_xy = ego_xy + ego_segment.vec.unit * 11

world_model.set_vehicle_state(EGO_ID, AutowareVehicleState(
    ego_xy.x, ego_xy.y + 993, ve, 0, ego_theta.r,
    shape
), VehicleMode.AUTOPILOT)
destinations['v1'] = {
    'x': -1.75, 'y': -900, 'theta': -math.pi / 2
}


world_model.set_vehicle_state(ARRIVING_ID, AutowareVehicleState(
    -1.75, 7 + xa + max(0, 65 - xa - xf), Context.vl_mps, 0, -math.pi / 2,
    shape
), VehicleMode.AUTOPILOT)
destinations['v2'] = {
    'x': -1.75, 'y': -900, 'theta': -math.pi / 2
}

front_y = 0


front_y = -xf - shape.back_edge_to_center
world_model.set_vehicle_state(FRONT_ID, AutowareVehicleState(
    -1.75, -xf - shape.back_edge_to_center, 0, 0, -math.pi / 2,
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

static_ticks = 0

def callback(simulator: AutowareSimulator, error_message):
    global front_state, collision, static_ticks
    ego_state = simulator.autoware_world_state.get_vehicle_state(EGO_ID)
    arriving_state = simulator.autoware_world_state.get_vehicle_state(
        ARRIVING_ID)
    ego_trace.append(ego_state.to_dict())
    arriving_trace.append(arriving_state.to_dict())
    front_state = simulator.autoware_world_state.get_vehicle_state(FRONT_ID)

    if simulator.collision:
        collision = True
        finish()
    
    if ego_state.v < 1e-5 and arriving_state.v < 1e-5:
        static_ticks += 1
    else:
        static_ticks = 0
        
    if static_ticks > 50:
        loguru.logger.success('Finish for all vehicle stop')
        finish()


config = {
    'carla_offset_x': 1000,
    'carla_offset_y': 0
}


server = AutowareSimulatorServer('server', 12346, world_model,
                                 destinations, opendrive_map_data, callback=callback, enable_carla=True, 
                                 collision_detected_actor=['v1', 'v2'], config=config)

server.run()
