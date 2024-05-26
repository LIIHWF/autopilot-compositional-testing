import sys
import math
import carla
import json
from carla import Transform, Location, Rotation  # type: ignore
import matplotlib.pyplot as plt
from src.simulator.carla import CarlaRunner, CarlaManager, VehicleMovementType
from src.simulator.adapter.argument import make_arg_parser


parser = make_arg_parser('carla', 'crossing_with_yield_signs')
experiment_args = parser.parse_args()


class CarlaCrossingRunner(CarlaRunner):
    def _get_opendrive_map(self) -> str:
        with open('map_data/carla/crossing_with_yield_signs.xodr') as f:
            return f.read()

    def _ego_transform_by_x(self, x):
        x = max(x, 0.1)
        assert self.args.ve is not None
        return Transform(Location(-12 - x - self.carla_manager.HALF_VEHICLE_LENGTH, 1.75, CarlaManager.INITIAL_Z))

    def _front_transform_by_x(self, x):
        return Transform(
            Location(12 + x + self.carla_manager.HALF_VEHICLE_LENGTH + self.carla_manager.LEADING_DISTANCE,
                     1.75, CarlaManager.INITIAL_Z))

    def _arriving_transform_by_x(self, x):
        x = max(x, 0.1)
        return Transform(
            Location(-1.75, -12 - x - self.carla_manager.HALF_VEHICLE_LENGTH,
                     CarlaManager.INITIAL_Z),
            Rotation(0, 90, 0)
        )



runner = CarlaCrossingRunner(experiment_args)


runner.init_vehicles(arriving_movement=VehicleMovementType.AUTOPILOT)

tick = 0


def finish():
    trace = {
        'xe': experiment_args.xe,
        've': experiment_args.ve,
        'xf': experiment_args.xf,
        'xa': experiment_args.xa,
        'state_sequence': state_sequence,
        'collision': 'yes' if result == 'collision' else 'no'
    }
    print(json.dumps(trace))
    sys.exit()


result = 'unknown'
state_sequence = []

ego = runner.ego
arriving = runner.arriving
front = runner.front
assert ego is not None and arriving is not None and front is not None


def actor_to_dict(actor: carla.Actor):
    if actor.get_location().x == 0.0 and actor.get_location().y == 0.0 and actor.get_location().z == 0.0:
        raise ValueError
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

tick = 0
for world in runner.tick():

    if runner.collision:
        result = 'collision'
        finish()

    tick += 1
    state_sequence.append({
        'ego': actor_to_dict(ego),
        'arriving': actor_to_dict(arriving),
        'front': actor_to_dict(front),
    })

    if front.get_location().distance(ego.get_location()) < 10 and ego.get_velocity().length() < 1e-5:
        break

    if arriving.get_location().y > 800:
        break

finish()
