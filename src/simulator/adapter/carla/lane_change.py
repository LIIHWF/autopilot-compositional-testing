import sys
import math
import carla
import json
from carla import Transform, Location, Rotation  # type: ignore
import matplotlib.pyplot as plt
from src.simulator.carla import CarlaRunner, CarlaManager, VehicleMovementType
from src.simulator.adapter.argument import make_arg_parser


parser = make_arg_parser('carla', 'lane_change')
experiment_args = parser.parse_args()

class CarlaLaneChangeRunner(CarlaRunner):
    THETA = math.pi / 6
    OFFSET = 3.5 / math.tan(THETA)

    def _get_opendrive_map(self) -> str:
        with open('map_data/carla/lane_change.xodr') as f:
            return f.read()

    def _ego_transform_by_x(self, x):
        return Transform(
            Location(1.75, -500, CarlaManager.INITIAL_Z),
            Rotation(yaw=-90),
        )

    def _obstacle_transform_by_x(self, x):
        assert self.args.ve is not None
        return Transform(
            Location(1.75, -500 - x - self.carla_manager.LEADING_DISTANCE - 0.2 - 2 * CarlaManager.HALF_VEHICLE_LENGTH -
                     self.args.ve * CarlaManager.FIXED_DELTA_SECONDS, CarlaManager.INITIAL_Z),
            Rotation(yaw=-90),
        )

    def _front_transform_by_x(self, x):
        return Transform(
            Location(-1.75,  -500 - self.OFFSET - x - 0.2 - 2 * self.carla_manager.HALF_VEHICLE_LENGTH - self.carla_manager.LEADING_DISTANCE,
                     CarlaManager.INITIAL_Z),
            Rotation(yaw=-90),
        )

    def _arriving_transform_by_x(self, x):
        return Transform(
            Location(-1.75, -500 - self.OFFSET + x - self.carla_manager.LEADING_DISTANCE,
                     CarlaManager.INITIAL_Z),
            Rotation(yaw=-90),
        )


runner = CarlaLaneChangeRunner(experiment_args)
runner.init_vehicles(arriving_movement=VehicleMovementType.AUTOPILOT,
                     obstacle_movement=VehicleMovementType.STATIC)
traffic_manager = runner.carla_manager.get_traffic_manager()
traffic_manager.auto_lane_change(runner.ego, True)
traffic_manager.auto_lane_change(runner.arriving, False)
traffic_manager.random_left_lanechange_percentage(runner.ego, 100)


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


state_sequence = []

def finish():
    trace = {
        'xf\'': experiment_args.xff,
        've': experiment_args.ve,
        'xf': experiment_args.xf,
        'xa': experiment_args.xa,
        'state_sequence': state_sequence,
        'collision': 'yes' if result == 'collision' else 'no'
    }
    print(json.dumps(trace))
    sys.exit()

ego = runner.ego
front = runner.front
arriving = runner.arriving
assert ego is not None and front is not None and arriving is not None

result = 'unknown'
tick = 0

static_cnt = 0

for world in runner.tick():
    state_sequence.append({
        'ego': actor_to_dict(ego),
        'front': actor_to_dict(front),
        'arriving': actor_to_dict(arriving),
    })
    
    if runner.collision:
        result = 'collision'
        break

    if ego.get_velocity().length() < 1e-5 and arriving.get_velocity().length() < 1e-5:
        static_cnt += 1
    else:
        static_cnt = 0
        
    if static_cnt > 100:
        break
    

finish()