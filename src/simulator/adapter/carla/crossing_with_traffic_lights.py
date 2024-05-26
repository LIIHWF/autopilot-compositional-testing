import sys
import math
import carla
import json
from carla import Transform, Location, Rotation  # type: ignore
import matplotlib.pyplot as plt
from src.simulator.carla import CarlaRunner, CarlaManager, VehicleMovementType
from src.simulator.adapter.argument import make_arg_parser


parser = make_arg_parser('carla', 'crossing_with_traffic_lights')
experiment_args = parser.parse_args()


class CarlaTrafficLightRunner(CarlaRunner):
    def _get_opendrive_map(self) -> str:
        with open('map_data/carla/crossing_with_traffic_lights.xodr') as f:
            return f.read()

    def _ego_transform_by_x(self, x):
        assert self.args.ve is not None
        return Transform(Location(
            -12 - x - self.carla_manager.HALF_VEHICLE_LENGTH - 0.2 - 3 * self.carla_manager.FIXED_DELTA_SECONDS * self.args.ve
            , 1.75, CarlaManager.INITIAL_Z))
    
    def _front_transform_by_x(self, x):
        return Transform(
            Location(12 + x + self.carla_manager.HALF_VEHICLE_LENGTH + self.carla_manager.LEADING_DISTANCE,
                      1.75, CarlaManager.INITIAL_Z))


runner = CarlaTrafficLightRunner(experiment_args)
assert runner.args.ve is not None

runner.carla_manager.world.tick()
self_traffic_light = runner.get_green_traffic_light()
side_traffic_light = runner.get_red_traffic_light()

runner.init_vehicles(arriving_movement=None)

init_frame=int(round((self_traffic_light.get_green_time()) / runner.carla_manager.FIXED_DELTA_SECONDS)) - 1


def actor_to_dict(actor: carla.Actor):
    return {
        'transform': {
            'location': {
                'x': actor.get_location().x,
                'y': actor.get_location().y,
                'z': actor.get_location().z
            },
            'rotation': {
                'pitch': actor.get_transform().rotation.pitch,
                'yaw': actor.get_transform().rotation.yaw,
                'roll': actor.get_transform().rotation.roll
            },
        },
        'velocity': {
            'x': actor.get_velocity().x,
            'y': actor.get_velocity().y,
            'z': actor.get_velocity().z
        },
        'shape': {
            'x': actor.bounding_box.extent.x,
            'y': actor.bounding_box.extent.y,
            'z': actor.bounding_box.extent.z
        }
    }


def finish():
    trace = {
        'xe': experiment_args.xe,
        've': experiment_args.ve,
        'xf': experiment_args.xf,
        'ego_trace': ego_trace,
        'front_state': actor_to_dict(runner.front),
        'self_traffic_light_trace': self_traffic_light_trace,
        'side_traffic_light_trace': side_traffic_light_trace
    }
    print(json.dumps(trace))
    sys.exit()


ego_trace = []
self_traffic_light_trace = []
side_traffic_light_trace = []


static_cnt = 0

for world in runner.tick(init_frame):
    ego_trace.append(actor_to_dict(runner.ego))
    self_traffic_light_trace.append(self_traffic_light.get_state().name)
    side_traffic_light_trace.append(side_traffic_light.get_state().name)
    
    if runner.ego.get_velocity().length() < 1e-5:
        static_cnt += 1
    else:
        static_cnt = 0
    if static_cnt > 50:
        finish()
