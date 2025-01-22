import carla
import os
import time
import loguru
import threading


def remove_spaces(text):
    return text.replace(' ', '').replace('\n', '').replace('\t', '').replace('\r', '')


def test_connection(address, port):
    try:
        client = carla.Client(address, port)
        world = client.get_world()
        world.tick()
        return client, world
    except RuntimeError:
        return False


def load_map(map_name_or_opendrive_map_str, address, port, speed_limit_kmh, fixed_delta_seconds, reload_required=True):
    client = carla.Client(address, port, worker_threads=1)
    client.set_timeout(20.0)
    if map_name_or_opendrive_map_str in client.get_available_maps():
        world_settings = client.get_world().get_settings()
        world_settings.synchronous_mode = True
        world_settings.fixed_delta_seconds = fixed_delta_seconds
        client.get_world().apply_settings(world_settings)
        client.load_world(map_name_or_opendrive_map_str, False)
    elif remove_spaces(client.get_world().get_map().to_opendrive()) == remove_spaces(map_name_or_opendrive_map_str):
        world_settings = client.get_world().get_settings()
        world_settings.synchronous_mode = True
        world_settings.fixed_delta_seconds = fixed_delta_seconds
        client.get_world().apply_settings(world_settings)
        # remove all vehicles and sensors
        if not reload_required:
            for actor in client.get_world().get_actors():
                if actor.type_id.startswith('vehicle') or actor.type_id.startswith('sensor'):
                    actor.destroy()
            loguru.logger.info('Reset world')
        else:
            client.reload_world(False)
            loguru.logger.info('Reloaded world')
    else:
        client.generate_opendrive_world(map_name_or_opendrive_map_str)
        world_settings = client.get_world().get_settings()
        world_settings.synchronous_mode = True
        world_settings.fixed_delta_seconds = fixed_delta_seconds
        client.get_world().apply_settings(world_settings)
        client.reload_world(False)


class CarlaManager:
    FIXED_DELTA_SECONDS = 0.05
    TRAFFIC_MANAGER_SEED = 1
    DEFAULT_VEHICLE_BP = 'vehicle.tesla.model3'
    HALF_VEHICLE_LENGTH = 2.3958897590637207
    DEFAULT_SPEED_LIMIT = 30
    INITIAL_Z = 0.013
    LEADING_DISTANCE = 1.5

    def __init__(self, map_name_or_opendrive_map, speed_limit_kmh, addr='localhost', port=2000, fixed_delta_seconds=None, reload_required=True):
        self.map_name_or_opendrive_map = map_name_or_opendrive_map
        self.speed_limit_kmh = speed_limit_kmh
        self.addr = addr
        self.port = port if 'CARLA_PORT' not in os.environ else int(os.environ['CARLA_PORT'])
        if fixed_delta_seconds is not None:
            self.fixed_delta_seconds = fixed_delta_seconds
        else:
            self.fixed_delta_seconds = self.FIXED_DELTA_SECONDS
        self.reload_required = reload_required
        self._connect_carla()

    def _connect_carla(self):
        loguru.logger.info('Connecting to Carla...')
        load_map(self.map_name_or_opendrive_map, self.addr, self.port,
                 self.speed_limit_kmh, self.fixed_delta_seconds, self.reload_required)
        ret = test_connection(self.addr, self.port)
        if ret:
            self.client, self.world = ret
            self.set_traffic_manager()
            failed = False
        else:
            raise ConnectionError('Failed to connect to Carla')

    def tick(self):
        self.world.tick()

    def set_traffic_manager(self):
        traffic_manager = self.client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(self.TRAFFIC_MANAGER_SEED)
        traffic_manager.global_percentage_speed_difference(
            100 - self.speed_limit_kmh / 30 * 103.8)
        traffic_manager.set_global_distance_to_leading_vehicle(
            self.LEADING_DISTANCE + 1)

    def get_traffic_manager(self):
        traffic_manager = self.client.get_trafficmanager()
        return traffic_manager

    def default_vehicle_bp(self):
        blueprint_lib = self.world.get_blueprint_library()
        bp = list(blueprint_lib.filter(self.DEFAULT_VEHICLE_BP))[0]
        return bp

    def collision_sensor_bp(self):
        blueprint_lib = self.world.get_blueprint_library()
        bp = blueprint_lib.find('sensor.other.collision')
        return bp

    def set_vehicle_transform(self, actor, transform: carla.Transform):
        actor.set_transform(transform)
        self.world.tick()

    def set_vehicle_velocity(self, actor, velocity: carla.Vector3D):
        # actor.set_target_velocity(velocity)
        actor.enable_constant_velocity(velocity)
        self.world.tick()
        actor.disable_constant_velocity()

    def set_gear_switch_time(self, actor, t):
        # use carla API
        physics_control = actor.get_physics_control()
        physics_control.gear_switch_time = t
        actor.apply_physics_control(physics_control)

    def get_initial_throttle_by_speed_kmh(self, speed_khm):
        epsilon = 1e-3
        if abs(speed_khm - 80) < epsilon:
            return 0.619124
        if abs(speed_khm - 120) < epsilon:
            return 0.724180
        raise NotImplementedError

    @classmethod
    def get_brake_distance(cls, speed_mps):
        epsilon = 1e-3
        data = {
            0: 0,
            5: 0.8,
            10: 6.8,
            15: 15.8,
            22: 30.9
        }
        for key, distance in data.items():
            if abs(speed_mps - key) < epsilon:
                return distance
        raise NotImplementedError

    @classmethod
    def get_initial_throttle_by_speed_mps(cls, speed_mps):
        epsilon = 1e-3
        data = {
            0:       0,
            4:       0.27,
            5:       0.28,
            6:       0.314,
            7:       0.401,
            8:       0.357,
            9:       0.429,
            10:      0.44332,
            11:      0.46875,
            12:      0.46875,
            13:      0.46875,
            14:      0.5,
            15:      0.52,
            16:      0.53125,
            17:      0.5625,
            18:      0.5625,
            19:      0.59375,
            20:      0.6,
            21:      0.609375,
            22:      0.625,
            22.2222: 0.625
        }
        for key, throttle in data.items():
            if abs(speed_mps - key) < epsilon:
                return throttle
        for key, throttle in data.items():
            if abs(speed_mps - key) <= 0.51:
                loguru.logger.warning(f'Approximated throttle for speed_mps = {speed_mps} as {key}')
                return throttle
        else:
            return 0.27

    def __del__(self):
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)
