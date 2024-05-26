from src.common.types import *
import math
import loguru
from src.simulator.carla import CarlaManager
from .autoware_world_model import AutowareWorldModel, TrafficLightColor, VehicleMode
from .autoware_vehicle_state import AutowareVehicleState
import carla


class AutowareSimulator:
    def __init__(self, initial_world_state: AutowareWorldModel, map_data: str, speed_limit: float,
                 collision_detected_actor: Optional[List] = None,
                 enable_carla: bool = True, config=None):
        self._autoware_world_state = initial_world_state
        self.collision = False
        self._enable_carla = enable_carla
        self.forbidden_vehicles = dict()
        self.config = config if config is not None else dict()
        if self._enable_carla:
            self._carla_manager = CarlaManager(map_data, speed_limit, reload_required=False)
            self._actors = dict()
            if collision_detected_actor is None:
                collision_detected_actor = []
            self._collision_detected_actor = collision_detected_actor
            self._collision_sensors = []
            self._init_carla()

    def _init_carla(self):
        self._carla_manager.FIXED_DELTA_SECONDS = 0.1
        self._carla_world = self._carla_manager.world
        carla_world = self._carla_world
        carla_settings = self._carla_world.get_settings()
        carla_settings.synchronous_mode = True
        carla_settings.fixed_delta_seconds = 0.1
        carla_settings.no_rendering_mode = False
        self._carla_world.apply_settings(carla_settings)

        carla_offset_x = 0
        carla_offset_y = 0
        
        if self.config is not None and 'carla_offset_x' in self.config and 'carla_offset_y' in self.config:
            carla_offset_x = self.config['carla_offset_x']
            carla_offset_y = self.config['carla_offset_y']

        flip = False
        if 'carla_flip' in self.config:
            flip = self.config['carla_flip']

        for vehicle_id, autoware_vehicle_state in self._autoware_world_state.vehicle_states.items():
            self._actors[vehicle_id] = carla_world.spawn_actor(
                self._carla_manager.default_vehicle_bp(),
                carla.Transform(carla.Location(
                    -(autoware_vehicle_state.geometry_center.y if not flip else -autoware_vehicle_state.geometry_center.x) + carla_offset_x, 
                    -(autoware_vehicle_state.geometry_center.x if not flip else -autoware_vehicle_state.geometry_center.y) + carla_offset_y,
                    self._carla_manager.INITIAL_Z),
                carla.Rotation(0, (1 if flip else 1) * (270 - autoware_vehicle_state.theta_degrees + (-270 if flip else 0)), 0)))
            if vehicle_id in self._collision_detected_actor:
                blueprint_library = self._carla_world.get_blueprint_library()
                self._collision_detected_actor.append(self._carla_world.spawn_actor(blueprint_library.find('sensor.other.collision'),
                                                                                    carla.Transform(carla.Location(), carla.Rotation()), attach_to=self._actors[vehicle_id]))
                self._collision_detected_actor[-1].listen(
                    lambda event: self._handle_collision())

    def _handle_collision(self):
        self.collision = True

    def get_vista_for_vehicle(self, vehicle_id: str) -> AutowareWorldModel:
        if vehicle_id in self.forbidden_vehicles:
            state = self.autoware_world_state.copy()
            for vid in self.forbidden_vehicles[vehicle_id]:
                state = state.remove_vehicle(vid)
            return state
        else:
            return self.autoware_world_state.copy()

    @property
    def autoware_world_state(self):
        return self._autoware_world_state

    @property
    def carla_manager(self):
        return self._carla_manager

    @property
    def frame(self) -> int:
        return self.autoware_world_state.frame

    def tick(self, new_autoware_vehicle_states: Mapping[str, AutowareVehicleState]):
        new_autoware_world_state = self._autoware_world_state
        for vehicle_id, autoware_vehicle_state in new_autoware_vehicle_states.items():
            new_autoware_world_state.set_vehicle_state(
                vehicle_id, autoware_vehicle_state)
        new_autoware_world_state.set_frame(self.frame + 1)
        if self._enable_carla:
            self._update_carla_state(new_autoware_world_state)
            self._carla_manager.tick()
        if self.collision:
            loguru.logger.error('Collision detected')

    def _update_carla_state(self, autoware_world_state: AutowareWorldModel):
        update_command = []
        for vehicle_id in autoware_world_state.vehicle_ids:
            # if autoware_world_state.get_vehicle_mode(vehicle_id) == VehicleMode.AUTOPILOT:
            carla_actor = self._actors[vehicle_id]
            actor_autoware_state = autoware_world_state.get_vehicle_state(
                vehicle_id)
            
            carla_offset_x = 0
            carla_offset_y = 0
            
            if self.config is not None and 'carla_offset_x' in self.config and 'carla_offset_y' in self.config:
                carla_offset_x = self.config['carla_offset_x']
                carla_offset_y = self.config['carla_offset_y']
                
            flip = False
            if 'carla_flip' in self.config:
                flip = self.config['carla_flip']

            cmd = carla.command.ApplyTransform(carla_actor.id,
                carla.Transform(
                    carla.Location(
                        -(actor_autoware_state.geometry_center.y if not flip else -actor_autoware_state.geometry_center.x) + carla_offset_x,
                        -(actor_autoware_state.geometry_center.x if not flip else actor_autoware_state.geometry_center.y) + carla_offset_y,
                        carla_actor.get_transform().location.z),
                    carla.Rotation(
                        0, (1 if flip else 1) * (270 - actor_autoware_state.theta_degrees + (-270 if flip else 0)), 0)
                ))
            update_command.append(cmd)
        # set spectator
        spectator = self._carla_world.get_spectator()
        ego_actor = self._actors['v1']
        transform = ego_actor.get_transform()
        transform.location.z = 100
        transform.rotation.roll = 0
        transform.rotation.yaw = 0
        transform.rotation.pitch = -90
        spectator.set_transform(transform)
        self.carla_manager.client.apply_batch_sync(update_command)
        
        if self._actors['v1'].get_location().z > 0.5:
            self.collision = True
