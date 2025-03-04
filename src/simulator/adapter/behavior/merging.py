import carla
import json
from carla import Transform, Location, Rotation  # type: ignore
from src.simulator.carla import CarlaRunner, CarlaManager, VehicleMovementType
from src.simulator.adapter.argument import make_arg_parser

from src.simulator.context import Context

Context.vl_kmh = 30
Context.vl_mps = 30 / 3.6

import sys
sys.path.append('carla/PythonAPI/carla')

from agents.navigation.behavior_agent import BehaviorAgent

parser = make_arg_parser('behavior', 'merging')
experiment_args = parser.parse_args()


class CarlaMergingRunner(CarlaRunner):
    MERGING_POINT_X = 996
    def _get_opendrive_map(self):
        with open('map_data/carla/merging-30.xodr') as f:
            return f.read()

    def _ego_transform_by_x(self, x):
        world_map = self.carla_manager.world.get_map()
        waypoint = world_map.get_waypoint_xodr(2, -1, 1.1102673540396728e+03 - x - 1)
        transform = waypoint.transform
        transform.location.z = CarlaManager.INITIAL_Z
        return transform
    
    def _front_transform_by_x(self, x):
        vehicle_half_length = self.carla_manager.HALF_VEHICLE_LENGTH
        return Transform(
                Location(self.MERGING_POINT_X + vehicle_half_length + x + CarlaManager.LEADING_DISTANCE, 1.75, CarlaManager.INITIAL_Z),
            )
    
    def _arriving_transform_by_x(self, x):
        vehicle_half_length = self.carla_manager.HALF_VEHICLE_LENGTH
        return Transform(
            Location(self.MERGING_POINT_X - x - vehicle_half_length * 2, 1.75, CarlaManager.INITIAL_Z),
        )


runner = CarlaMergingRunner(experiment_args)
runner.init_vehicles(ego_movement=VehicleMovementType.MANUAL, arriving_movement=VehicleMovementType.MANUAL)
traffic_manager = runner.carla_manager.get_traffic_manager()


tick = 0

ego = runner.ego
front = runner.front
arriving = runner.arriving

assert ego is not None and front is not None and arriving is not None


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

ego_agent = BehaviorAgent(
    ego,
    behavior="normal"
)
ego_agent.set_destination(
    start_location=ego.get_location(),
    end_location=runner._front_transform_by_x(800).location
)

arriving_agent = BehaviorAgent(
    arriving,
    behavior="normal"
)
arriving_agent.set_destination(
    start_location=arriving.get_location(),
    end_location=runner._front_transform_by_x(800).location
)

ego_control = ego_agent.run_step()
arriving_control = arriving_agent.run_step()
ego.apply_control(ego_control)
arriving.apply_control(arriving_control)

for world in runner.tick():
    
    state_sequence.append({
        'ego': actor_to_dict(ego),
        'front': actor_to_dict(front),
        'arriving': actor_to_dict(arriving),
    })
    
    ego_control = ego_agent.run_step()
    arriving_control = arriving_agent.run_step()
    ego.apply_control(ego_control)
    arriving.apply_control(arriving_control)
    
    if (ego.get_velocity().length() < 1e-5 and 
            arriving.get_velocity().length() < 1e-5) and front.get_transform().location.distance(ego.get_transform().location) < 25:
        break

    if runner.collision:        
        result = 'collision'
        break
    
finish()