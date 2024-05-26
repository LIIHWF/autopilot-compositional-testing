from src.common.types import *
from src.semantic_model.geometry import *
from .apollo_world_model import ApolloWorldModel, TrafficLightColor
from .apollo_vehicle_state import ApolloVehicleState
import carla


def remove_spaces(text):
    return text.replace(' ', '').replace('\n', '').replace('\t', '').replace('\r', '')


class CarlaConfiguration:
    def __init__(self, server_address: str, server_port: int, opendrive_map_data: str):
        self._server_address = server_address
        self._server_port = server_port
        self._opendrive_map_data = opendrive_map_data

        self.carla_actors = dict()
        self.carla_world: Optional[carla.World] = None

    @property
    def server_address(self) -> str:
        return self._server_address

    @property
    def server_port(self) -> int:
        return self._server_port

    @property
    def opendrive_map_data(self) -> str:
        return self._opendrive_map_data


class ApolloSimulator:
    def __init__(
        self,
        initial_world_state: ApolloWorldModel,
        carla_configuration: Optional[CarlaConfiguration] = None,
        fixed_delta_seconds=0.1,
        has_traffic_light = False,
        other_config = None
    ):
        self._world_state: ApolloWorldModel = initial_world_state

        self._carla_configuration = carla_configuration

        self._collision = False

        self._fixed_delta_seconds = fixed_delta_seconds

        self._ignore_vehicles_map = dict()
        
        self._other_config = other_config

        self._has_traffic_light = has_traffic_light

        if self._carla_configuration:
            self._init_carla()

    @property
    def ignore_vehicles_map(self) -> Dict[str, Set[str]]:
        return self._ignore_vehicles_map

    def _init_carla(self):
        assert self._carla_configuration is not None
        config = self._carla_configuration
        carla_client = carla.Client(config.server_address, config.server_port)

        if remove_spaces(carla_client.get_world().get_map().to_opendrive()) != remove_spaces(config.opendrive_map_data):
            carla_client.generate_opendrive_world(config.opendrive_map_data)

        carla_world = carla_client.get_world()
        carla_settings = carla_world.get_settings()
        carla_settings.synchronous_mode = True
        carla_settings.fixed_delta_seconds = self._fixed_delta_seconds
        carla_world.apply_settings(carla_settings)

        # carla_client.reload_world(False)
        for actor in carla_world.get_actors():
            if actor.type_id.startswith('vehicle') or actor.type_id.startswith('sensor'):
                actor.destroy()

        traffic_manager = carla_client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(1)

        self._carla_configuration.carla_world = carla_world

        self._setup_carla_world()

    @property
    def collision(self) -> bool:
        return self._collision

    @property
    def frame(self) -> int:
        return self.world_state.frame

    @property
    def carla_configuration(self) -> Optional[CarlaConfiguration]:
        return self._carla_configuration

    @property
    def autopilot_ids(self):
        return self.world_state.autopilot_ids

    @property
    def vehicle_ids(self):
        return self.world_state.vehicle_ids

    @property
    def world_state(self) -> ApolloWorldModel:
        assert self._world_state is not None
        return self._world_state

    def vehicle_state(self, vehicle_id: str) -> ApolloVehicleState:
        return self._world_state.get_vehicle_state(vehicle_id)

    def get_partial_world_state(self, vehicle_id: str) -> ApolloWorldModel:
        world_state = self.world_state.copy()
        return self.world_state

    def tick(self, apollo_vehicle_states: Mapping[str, ApolloVehicleState]):
        for uid, v_state in apollo_vehicle_states.items():
            self._world_state.set_vehicle_state(uid, v_state)

        self._world_state.set_frame(self._world_state.frame + 1)

        if self.carla_configuration is not None:
            self._update_carla_world(self._world_state)
            if self._has_traffic_light:
                self._set_traffic_light()
                
    def _set_traffic_light(self):
        assert self._has_traffic_light
        assert self.carla_configuration is not None and self.carla_configuration.carla_world is not None
        for tid in ['sign_0', 'sign_1', 'sign_2', 'sign_3']:
            if tid.endswith('0') or tid.endswith('2'):
                carla_color = self.carla_configuration.carla_world.get_traffic_light_from_opendrive_id('30').get_state()
            else:
                carla_color = self.carla_configuration.carla_world.get_traffic_light_from_opendrive_id('32').get_state()
            if carla_color == carla.TrafficLightState.Green:
                self.world_state.set_traffic_light_state(tid, TrafficLightColor.GREEN)
            if carla_color == carla.TrafficLightState.Red:
                self.world_state.set_traffic_light_state(tid, TrafficLightColor.RED)
            if carla_color == carla.TrafficLightState.Yellow:
                self.world_state.set_traffic_light_state(tid, TrafficLightColor.YELLOW)

    def _setup_carla_world(self):
        assert (
            self.carla_configuration is not None
            and self.carla_configuration.carla_world is not None
        )
        carla_world = self.carla_configuration.carla_world
        carla_actors = self.carla_configuration.carla_actors

        for vid in self._world_state.vehicle_ids:
            v_state = self._world_state.get_vehicle_state(vid)
            transform = carla.Transform()
            
            carla_offset_x = 0
            carla_offset_y = 0
            
            if self._other_config is not None and 'carla_offset_x' in self._other_config and 'carla_offset_y' in self._other_config:
                carla_offset_x = self._other_config['carla_offset_x']
                carla_offset_y = self._other_config['carla_offset_y']
            
            transform.location.x = -v_state.vehicle_center.y + carla_offset_x
            transform.location.y = -v_state.vehicle_center.x + carla_offset_y
            transform.location.z = 10  # 0.05
            transform.rotation.yaw = 270 - v_state.theta_degrees
            
            blueprint_library = carla_world.get_blueprint_library()
            bp = next(iter(blueprint_library.filter("vehicle.lincoln.mkz_2017")))
            carla_actors[vid] = carla_world.spawn_actor(bp, transform)

            if vid == "apollo-1":
                self.carla_configuration.carla_actors[vid + "-collision"] = (
                    carla_world.spawn_actor(
                        blueprint_library.find("sensor.other.collision"),
                        carla.Transform(),
                        attach_to=carla_actors[vid],
                    )
                )

                carla_actors[vid + "-collision"].listen(
                    lambda event: self._handle_collision()
                )
        self._set_carla_spectator()
        
        if self._has_traffic_light:
            green_time = 20
            for actor in carla_world.get_actors():
                if isinstance(actor, carla.TrafficLight):
                    actor.set_green_time(green_time)
            carla_world.tick()
            carla_world.reset_all_traffic_lights()
            for _ in range(int(green_time / self._fixed_delta_seconds) - 1):
                carla_world.tick()
            self._set_traffic_light()
        
    def _handle_collision(self):
        self._collision = True

    def _update_carla_world(self, apollo_world_state: ApolloWorldModel):
        assert (
            self.carla_configuration is not None
            and self.carla_configuration.carla_world is not None
        )
        for vid in apollo_world_state.vehicle_ids:
            v_state = apollo_world_state.get_vehicle_state(vid)
            transform = self.carla_configuration.carla_actors[vid].get_transform(
            )
            
            carla_offset_x = 0
            carla_offset_y = 0
            
            if self._other_config is not None and 'carla_offset_x' in self._other_config and 'carla_offset_y' in self._other_config:
                carla_offset_x = self._other_config['carla_offset_x']
                carla_offset_y = self._other_config['carla_offset_y']
            
            transform.location.x = -v_state.vehicle_center.y + carla_offset_x
            transform.location.y = -v_state.vehicle_center.x + carla_offset_y
            transform.rotation.yaw = 270 - v_state.theta_degrees
            self.carla_configuration.carla_actors[vid].set_transform(transform)
        self._set_carla_spectator()
        self.carla_configuration.carla_world.tick()

    def _set_carla_spectator(self):
        # make carla spectator follow the ego vehicle
        assert (
            self.carla_configuration is not None
            and self.carla_configuration.carla_world is not None
        )
        ego_vehicle = self.carla_configuration.carla_actors["apollo-1"]
        transform = ego_vehicle.get_transform()
        transform.location.z = 100
        transform.rotation.roll = 0
        transform.rotation.yaw = 0
        transform.rotation.pitch = -90
        self.carla_configuration.carla_world.get_spectator().set_transform(transform)

    def get_apollo_world_state_dict(self):
        return self._world_state.to_dict()
