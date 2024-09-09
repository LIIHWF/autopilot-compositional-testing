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
        # data = {
        #     0: 0,
        #     5: 1.4,
        #     6: 1.6,
        #     7: 2.3,
        #     8: 3.7,
        #     9: 5.1,
        #     10: 6.6,
        #     11: 8.2,
        #     12: 10.0,
        #     13: 11.7,
        #     14: 13.6,
        #     15: 15.6,
        #     16: 17.6,
        #     17: 19.7,
        #     18: 21.9,
        #     19: 24.1,
        #     20: 26.2,
        #     21: 28.5,
        #     22: 30.7,
        # }
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
        # data = {
        #     0: 0,
        #     5: 0.27,
        #     6: 0.37168481945991516,
        #     7: 0.3842879831790924,
        #     8: 0.39901289343833923,
        #     9: 0.4150502383708954,
        #     10: 0.43,
        #     11: 0.4490894675254822,
        #     12: 0.4664364457130432,
        #     13: 0.4786297082901001,
        #     14: 0.49992409348487854,
        #     15: 0.52,
        #     16: 0.5353872179985046,
        #     17: 0.5568480491638184,
        #     18: 0.5723517537117004,
        #     19: 0.5869798064231873,
        #     20: 0.60,
        #     22.222222: 0.619124,
        #     25: 0.66,
        #     30: 0.70,
        #     33.333333: 0.724180
        # }
        # data = {
        #     0:       0,
        #     5:       0.252,
        #     6:       0.35023586452007294,
        #     7:       0.3675491511821747,
        #     8:       0.38580547273159027,
        #     9:       0.40464772284030914,
        #     10:      0.42380473017692566,
        #     11:      0.4430517703294754,
        #     12:      0.46221648156642914,
        #     13:      0.4813607633113861,
        #     14:      0.49992649257183075,
        #     15:      0.518090009689331,
        #     16:      0.5353865623474121,
        #     17:      0.5568480491638184,
        #     18:      0.5723517537117004,
        #     19:      0.5869817733764648,
        #     20:      0.6008298993110657,
        #     21:      0.6139577627182007,
        #     22:      0.6264127492904663,
        #     23:      0.6382463574409485,
        #     24:      0.6495137810707092,
        #     25:      0.660241425037384,
        #     26:      0.6704944968223572,
        #     27:      0.680264413356781,
        #     28:      0.6896273195743561,
        #     29:      0.6985878348350525,
        #     30:      0.7071617841720581,
        #     31:      0.7154030799865723,
        #     32:      0.7233148217201233,
        #     33:      0.7309064269065857
        # }
        data = {
            0:       0,
            # 5:       0.25279,
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
        raise NotImplementedError(f'speed_mps = {speed_mps}')

    def __del__(self):
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)
