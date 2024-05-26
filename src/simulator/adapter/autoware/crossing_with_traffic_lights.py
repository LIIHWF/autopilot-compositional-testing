from src.simulator.autoware import *
import math
import matplotlib.pyplot as plt
import loguru
import json
import os, sys
import argparse
import time
import carla

from src.simulator.adapter.argument import make_arg_parser
from src.simulator.context import Context

os.system('script/switch_autoware_map.sh crossing_with_traffic_lights')
args = make_arg_parser('autoware', 'crossing_with_traffic_lights').parse_args()


world_model = AutowareWorldModel()
destinations = dict()

shape = AutowareVehicleShape(3.79, 1.1, 0.948, 0.948, 2.5)

xe = args.xe
ve = args.ve
xf = args.xf


EGO_ID = 'v1'
FRONT_ID = 'front'

world_model.set_vehicle_state(EGO_ID, AutowareVehicleState(
    1.75, -12 - xe - shape.front_edge_to_center, ve, 0, math.pi / 2,
    shape
), VehicleMode.AUTOPILOT)
destinations[EGO_ID] = {
    'x': 1.75, 'y': 800, 'theta': math.pi / 2
}

world_model.set_vehicle_state(FRONT_ID, AutowareVehicleState(
    1.75, 12 + xf + 5 + shape.back_edge_to_center, 0, 0, math.pi / 2,
    shape
), VehicleMode.STATIC)


with open('map_data/carla/crossing_with_traffic_lights.xodr') as f:
    map_data = f.read()

accs = []


ego_trace = []
self_light_trace = []
side_light_trace = []
front_state = None


def finish():
    global collision
    assert front_state is not None
    print(json.dumps({
        'xe': xe,
        've': ve,
        'xf': xf,
        'ego_trace': ego_trace,
        'self_light_trace': self_light_trace,
        'side_light_trace': side_light_trace,
        'front_state': front_state.to_dict(),
        'collision': 'yes' if collision else 'no'
    }), flush=True)
    # exit()
    # time.sleep(0.5)
    os._exit(0)


first_pass = None
unsafe = False

collision = False

autoware_light_id = '1377'

stop_duration = 0

def carla_color_to_str(color):
    if color == carla.TrafficLightState.Green:
        return 'Green'
    elif color == carla.TrafficLightState.Red:
        return 'Red'
    elif color == carla.TrafficLightState.Yellow:
        return 'Yellow'
    else:
        return 'Unknown'


def callback(simulator: AutowareSimulator, error_message: str):
    global front_state, stop_duration
    ego_state = simulator.autoware_world_state.get_vehicle_state(EGO_ID)
    ego_trace.append(ego_state.to_dict())
    front_state = simulator.autoware_world_state.get_vehicle_state(FRONT_ID)

    if self_traffic_light.state == carla.TrafficLightState.Green:
        simulator.autoware_world_state.set_traffic_light_state(autoware_light_id, TrafficLightColor.GREEN)
    elif self_traffic_light.state == carla.TrafficLightState.Red:
        simulator.autoware_world_state.set_traffic_light_state(autoware_light_id, TrafficLightColor.RED)
    elif self_traffic_light.state == carla.TrafficLightState.Yellow:
        simulator.autoware_world_state.set_traffic_light_state(autoware_light_id, TrafficLightColor.YELLOW)

    self_light_trace.append(carla_color_to_str(self_traffic_light.state))
    side_light_trace.append(carla_color_to_str(side_traffic_light.state))

    if ego_state.v < 1e-5:
        stop_duration += 1
    else:
        stop_duration = 0

    if stop_duration > 30:
        finish()

    if ego_state.v < 1e-5 and front_state.y - ego_state.y < 15:
        finish()
    
    if simulator.frame > 600:
        finish()


server = AutowareSimulatorServer('server', 12346, world_model,
                                 destinations, map_data, callback=callback, enable_carla=True, collision_detected_actor=['v1'])


carla_world = server.simulator.carla_manager.world
self_traffic_light = carla_world.get_traffic_light_from_opendrive_id('30')
side_traffic_light = carla_world.get_traffic_light_from_opendrive_id('32')
carla_world.reset_all_traffic_lights()

green_time = 20
for actor in carla_world.get_actors():
    if isinstance(actor, carla.TrafficLight):
        actor.set_green_time(green_time)
carla_world.tick()
carla_world.reset_all_traffic_lights()
for _ in range(int(green_time / 0.1) - 1):
    carla_world.tick()

world_model.set_traffic_light_state('1377', TrafficLightColor.GREEN)

# green_time = 20
# for actor in carla_world.get_actors():
#     if isinstance(actor, carla.TrafficLight):
#         actor.set_green_time(green_time)
# carla_world.tick()
# carla_world.reset_all_traffic_lights()
# for _ in range(int(green_time / 0.1) + 50 - 1):
#     carla_world.tick()

# world_model.set_traffic_light_state('1377', TrafficLightColor.RED)

ego_trace.append(world_model.get_vehicle_state(EGO_ID).to_dict())
self_light_trace.append(carla_color_to_str(self_traffic_light.state))
side_light_trace.append(carla_color_to_str(side_traffic_light.state))

server.run()
