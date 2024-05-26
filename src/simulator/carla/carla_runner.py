from typing import Any
from src.common.types import *
import carla
from argparse import Namespace
import loguru
from carla import Transform, VehicleControl, Location, Rotation  # type: ignore
from .carla_manager import CarlaManager
from src.simulator.context import Context


SpawnActor = carla.command.SpawnActor  # type: ignore
ApplyTransform = carla.command.ApplyTransform  # type: ignore
ApplyTargetVelocity = carla.command.ApplyTargetVelocity  # type: ignore
SetAutopilot = carla.command.SetAutopilot  # type: ignore
ApplyVehicleControl = carla.command.ApplyVehicleControl  # type: ignore
ApplyVehiclePhysicsControl = carla.command.ApplyVehiclePhysicsControl  # type: ignore
TrafficLightState = carla.TrafficLightState  # type: ignore
ApplyVehicleAckermannControl = carla.command.ApplyVehicleAckermannControl  # type: ignore


class VehicleMovementType(Enum):
    STATIC = 0
    CONSTANT_SPEED = 1
    AUTOPILOT = 2
    MANUAL = 3


class CarlaRunner(ABC):
    def __init__(self, args: Namespace):
        self._args = args
        self._constant_vehicles = dict()
        self._ego = None
        self._front = None
        self._arriving = None
        self._obstacle = None
        self._collide_actor = None
        self._collision = False
        self._all_setting_commands = None
        self._init()

    def _init(self):
        self._carla_manager = CarlaManager(self._get_opendrive_map(), Context.vl_kmh)
        self._world = self._carla_manager.world
        self._client = self._carla_manager.client

    @property
    def collision(self):
        return self._collision

    @property
    def collide_actor(self):
        return self._collide_actor

    @property
    def args(self):
        return self._args

    @property
    def ego(self):
        assert self._ego is not None, "Ego vehicle is not initialized"
        return self._ego

    @property
    def arriving(self):
        assert self._arriving is not None, "Arriving vehicle is not initialized"
        return self._arriving

    @property
    def front(self):
        assert self._front is not None, "Front vehicle is not initialized"
        return self._front

    @property
    def obstacle(self):
        return self._obstacle

    @property
    def carla_manager(self):
        return self._carla_manager

    def _on_collision(self, event):
        self._collide_actor = event.other_actor
        self._collision = True

    def generate_spawn_vehicle_command(self, actor_transform: Transform):
        return SpawnActor(
            self.carla_manager.default_vehicle_bp(),
            Transform(actor_transform))

    def _set_autopilot(self, actor, initial_transform, initial_speed, initial_throttle):
        actor_control = VehicleControl()
        actor_control.gear = 1
        actor_control.throttle = \
            self.carla_manager.get_initial_throttle_by_speed_mps(
                initial_speed) if initial_throttle is None else initial_throttle
        actor_control.manual_gear_shift = True

        self._carla_manager.get_traffic_manager().set_route(actor,
                                                            ['Straight'])

        actor_transform = actor.get_transform()
        initial_location = initial_transform.location
        initial_location -= initial_transform.get_forward_vector() * initial_speed * \
            CarlaManager.FIXED_DELTA_SECONDS
        actor_transform.location.x = initial_location.x
        actor_transform.location.y = initial_location.y
        actor_transform.rotation.yaw = initial_transform.rotation.yaw

        return [
            ApplyVehicleControl(actor.id, actor_control),
            ApplyTransform(actor.id, actor_transform),
            ApplyTargetVelocity(
                actor.id, initial_transform.get_forward_vector() * initial_speed)
        ], [SetAutopilot(actor.id, True)]

    def _set_static(self, actor, initial_transform):
        traffic_manager = self.carla_manager.get_traffic_manager()
        traffic_manager.vehicle_percentage_speed_difference(actor, 100)
        return [
            SetAutopilot(actor.id, True),
            ApplyTransform(actor.id, initial_transform)
        ]

    def _set_constant_speed(self, actor, initial_transform, speed):
        self._constant_vehicles[actor.id] = initial_transform.get_forward_vector(
        ) * speed
        return [
            ApplyTargetVelocity(actor.id, self._constant_vehicles[actor.id]),
            ApplyTransform(actor.id, initial_transform)
        ]

    def _set_manual(self, actor, initial_transform, initial_speed):
        actor_control = VehicleControl()
        actor_control.gear = 1
        actor_control.throttle = self.carla_manager.get_initial_throttle_by_speed_mps(
            initial_speed)
        actor_control.manual_gear_shift = True

        return [
            SetAutopilot(actor.id, False),
            ApplyTransform(actor.id, initial_transform),
            ApplyTargetVelocity(
                actor.id, initial_transform.get_forward_vector() * initial_speed),
            ApplyVehicleControl(actor.id, actor_control),
        ]

    def _set_movement_mode(self, actor, movement_type: VehicleMovementType, initial_transform: Transform, initial_speed: Optional[Number] = None, initial_throttle=None):
        autopilot_commands = []
        if movement_type == VehicleMovementType.AUTOPILOT:
            setting_commands, autopilot_commands = self._set_autopilot(
                actor, initial_transform, initial_speed=initial_speed, initial_throttle=initial_throttle)
        elif movement_type == VehicleMovementType.CONSTANT_SPEED:
            setting_commands = self._set_constant_speed(
                actor, initial_transform, initial_speed)
        elif movement_type == VehicleMovementType.STATIC:
            setting_commands = self._set_static(actor, initial_transform)
        elif movement_type == VehicleMovementType.MANUAL:
            setting_commands = self._set_manual(
                actor, initial_transform, initial_speed)
        else:
            raise NotImplementedError
        return setting_commands, autopilot_commands

    @property
    def map(self):
        return self._get_map()

    @abstractmethod
    def _get_opendrive_map(self) -> str:
        raise NotImplementedError

    @abstractmethod
    def _ego_transform_by_x(self, x) -> Transform:
        raise NotImplementedError

    @abstractmethod
    def _front_transform_by_x(self, x) -> Transform:
        raise NotImplementedError

    def _arriving_transform_by_x(self, x) -> Transform:
        raise NotImplementedError

    def _obstacle_transform_by_x(self, x) -> Transform:
        raise NotImplementedError

    def get_green_traffic_light(self):
        for traffic_light in self.carla_manager.world.get_actors():
            if traffic_light.type_id == 'traffic.traffic_light' and traffic_light.get_state() == TrafficLightState.Green:
                return traffic_light
        raise RuntimeError

    def get_red_traffic_light(self):
        for traffic_light in self.carla_manager.world.get_actors():
            if traffic_light.type_id == 'traffic.traffic_light' and traffic_light.get_state() == TrafficLightState.Red:
                return traffic_light
        raise RuntimeError

    def init_vehicles(self,
                      ego_movement: Optional[VehicleMovementType] = VehicleMovementType.AUTOPILOT,
                      front_movement: Optional[VehicleMovementType] = VehicleMovementType.STATIC,
                      arriving_movement: Optional[VehicleMovementType] = VehicleMovementType.AUTOPILOT,
                      obstacle_movement: Optional[VehicleMovementType] = None,
                      initial_throttle=None):
        all_setting_commands = []
        all_autopilot_commnads = []

        if ego_movement is not None:
            ego_initial_transform = self._ego_transform_by_x(self.args.xe if 'xe' in self.args else self.args.xff)
            ego_blueprint = self.carla_manager.default_vehicle_bp()
            ego_blueprint.set_attribute('color', '0,0,255')
            self._ego = self.carla_manager.world.spawn_actor(
                ego_blueprint,
                ego_initial_transform
            )

            self._collision_sensor = self.carla_manager.world.spawn_actor(
                self.carla_manager.collision_sensor_bp(),
                Transform(),
                attach_to=self._ego
            )
            self._collision_sensor.listen(
                lambda event: self._on_collision(event))

            setting_commands, autopilot_commands = self._set_movement_mode(
                self._ego, ego_movement, ego_initial_transform, self.args.ve, initial_throttle)
            all_setting_commands.extend(setting_commands)
            all_autopilot_commnads.extend(autopilot_commands)

            spectator = self.carla_manager.world.get_spectator()
            spectator.set_transform(Transform(Location(
                ego_initial_transform.location.x, ego_initial_transform.location.y, 100), Rotation(-90, 0, 0)))

        if front_movement is not None:
            front_initial_transform = self._front_transform_by_x(self.args.xf)
            front_blueprint = self.carla_manager.default_vehicle_bp()
            front_blueprint.set_attribute('color', '0,255,0')
            self._front = self.carla_manager.world.spawn_actor(
                front_blueprint,
                front_initial_transform
            )
            setting_commands, autopilot_commands = self._set_movement_mode(
                self._front, front_movement, front_initial_transform, 0.0, initial_throttle)
            all_setting_commands.extend(setting_commands)
            all_autopilot_commnads.extend(autopilot_commands)

        if arriving_movement is not None:
            arriving_initial_transform = self._arriving_transform_by_x(
                self.args.xa)
            arriving_blueprint = self.carla_manager.default_vehicle_bp()
            arriving_blueprint.set_attribute('color', '255,0,0')
            self._arriving = self.carla_manager.world.spawn_actor(
                arriving_blueprint,
                arriving_initial_transform
            )
            setting_commands, autopilot_commands = self._set_movement_mode(
                self._arriving, arriving_movement, arriving_initial_transform, Context.vl_mps, initial_throttle)
            all_setting_commands.extend(setting_commands)
            all_autopilot_commnads.extend(autopilot_commands)

        if obstacle_movement is not None:
            obstacle_initial_transform = self._obstacle_transform_by_x(
                self.args.xe if 'xe' in self.args else self.args.xff)
            self._obstacle = self.carla_manager.world.spawn_actor(
                self.carla_manager.default_vehicle_bp(),
                obstacle_initial_transform
            )
            setting_commands, autopilot_commands = self._set_movement_mode(
                self._obstacle, obstacle_movement, obstacle_initial_transform, 0.0, initial_throttle)
            all_setting_commands.extend(setting_commands)
            all_autopilot_commnads.extend(autopilot_commands)

        self.carla_manager.client.apply_batch_sync(
            all_autopilot_commnads, False)
        acc_abs_history = []
        height_history = []
        for _ in range(int(5 / self.carla_manager.FIXED_DELTA_SECONDS)):
            responses = self.carla_manager.client.apply_batch_sync(
                all_setting_commands, True)
            acc_abs_history.append(self.ego.get_acceleration().length())
            height_history.append(self.ego.get_location().z)
            assert all([res.error == '' for res in responses])
            if all([vehicle is None or vehicle.get_acceleration().length() < 1e-5 for vehicle in (self._ego, self._front, self._arriving)]):
                break
            if len(acc_abs_history) > 10 and max(acc_abs_history[-10:]) - min(acc_abs_history[-10:]) < 1e-5 and \
                    len(height_history) > 10 and max(height_history[-10:]) - min(height_history[-10:]) < 1e-5:
                break
        # else:
        #     loguru.logger.warning("Could not initialize vehicles' acceleration")

        self._all_setting_commands = all_setting_commands

    def tick(self, init_frame=0):
        self.carla_manager.world.reset_all_traffic_lights()
        tick = 0
        while tick < init_frame:
            responses = self.carla_manager.client.apply_batch_sync(
                self._all_setting_commands, True)
            assert all([res.error == '' for res in responses])
            tick += 1

        while not self.collision:
            yield self.carla_manager.world
            constant_speed_commands = [
                ApplyTargetVelocity(actor_id, velocity)
                for actor_id, velocity in self._constant_vehicles.items()
            ]
            responses = self.carla_manager.client.apply_batch_sync(
                constant_speed_commands, False)
            self.carla_manager.world.tick()
            assert all([res.error == '' for res in responses])
        yield self.carla_manager.world
