import math
from loguru import logger

from typing import Optional, Mapping, Any

from bridge_common.vehicle_state import VehicleState, transform_to_vrf
from bridge_common.world_model import WorldState, TrafficLightColor

from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis
from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from modules.common_msgs.basic_msgs.pnc_point_pb2 import TrajectoryPoint
from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles
from cyber.proto.clock_pb2 import Clock
from modules.common_msgs.basic_msgs.error_code_pb2 import ErrorCode
from modules.common_msgs.perception_msgs.traffic_light_detection_pb2 import TrafficLightDetection, TrafficLight


def normalize_angle(angle):
    a = (angle + math.pi) % (2 * math.pi)
    if a < 0.0:
        a += 2 * math.pi
    return a - math.pi


def lerp_t(x0, t0, x1, t1, t):
    if abs(t1 - t0) < 1e-6:
        return x0
    r = (t - t0) / (t1 - t0)
    x = x0 + r * (x1 - x0)
    return x


def slerp_t(a0, t0, a1, t1, t):
    if abs(t1 - t0) < 1e-6:
        return normalize_angle(a0)
    a0_n = normalize_angle(a0)
    a1_n = normalize_angle(a1)
    d = a1_n - a0_n
    if d > math.pi:
        d = d - 2 * math.pi
    elif d < -math.pi:
        d = d + 2 * math.pi

    r = (t - t0) / (t1 - t0)
    a = a0_n + d * r
    return normalize_angle(a)


def linear_interpolation(tp0, tp1, t):
    pp0 = tp0.path_point
    pp1 = tp1.path_point
    t0 = tp0.relative_time
    t1 = tp1.relative_time

    tp = TrajectoryPoint()
    tp.v = lerp_t(tp0.v, t0, tp1.v, t1, t)
    tp.a = lerp_t(tp0.a, t0, tp1.a, t1, t)
    tp.relative_time = t
    tp.steer = slerp_t(tp0.steer, t0, tp1.steer, t1, t)

    path_point = tp.path_point
    path_point.x = lerp_t(pp0.x, t0, pp1.x, t1, t)
    path_point.y = lerp_t(pp0.y, t0, pp1.y, t1, t)
    path_point.theta = lerp_t(pp0.theta, t0, pp1.theta, t1, t)
    path_point.kappa = lerp_t(pp0.kappa, t0, pp1.kappa, t1, t)
    path_point.dkappa = lerp_t(pp0.dkappa, t0, pp1.dkappa, t1, t)
    path_point.ddkappa = lerp_t(pp0.ddkappa, t0, pp1.ddkappa, t1, t)
    path_point.s = lerp_t(pp0.s, t0, pp1.s, t1, t)
    return tp


def trajectory_point_to_vehicle_state(tp, shape) -> VehicleState:
    state = VehicleState(
        tp.path_point.x,
        tp.path_point.y,
        tp.v,
        tp.a,
        tp.path_point.theta,
        tp.path_point.kappa,
        shape
    )
    return state


def vehicle_state_to_trajectory_point(vehicle_state: VehicleState):
    point = TrajectoryPoint()
    point.path_point.x = vehicle_state.x
    point.path_point.y = vehicle_state.y
    point.path_point.theta = vehicle_state.theta_radians
    point.path_point.kappa = vehicle_state.kappa
    point.v = vehicle_state.v
    point.a = vehicle_state.a
    return point


class Vehicle:
    def __init__(self, uid: str, channels: Mapping[str, Any], on_state_updated):
        self._uid = uid
        self._world_state = WorldState()
        self.world_state.set_frame(0)
        self._fixed_delta_seconds: Optional[float] = 0

        self._vehicle_chassis_writer = channels['vehicle_chassis_writer']
        self._vehicle_pose_writer = channels['vehicle_pose_writer']
        self._localization_status_writer = channels['localization_status_writer']
        self._tf_writer = channels['tf_writer']
        self._obstacles_writer = channels['obstacles_writer']
        self._clock_writer = channels['clock_writer']
        self._traffic_light_writer = channels['traffic_light_writer']
        self._trajectory_reader = channels['trajectory_reader']

        self._on_state_updated = on_state_updated

        self._gear_position = Chassis.GEAR_NEUTRAL
        self._prev_point = None

    @property
    def world_state(self):
        return self._world_state

    def set_world_state(self, world_state: WorldState):
        self._world_state = world_state

    @property
    def uid(self):
        return self._uid

    @property
    def frame(self) -> int:
        return self.world_state.frame

    @property
    def state(self) -> VehicleState:
        return self.world_state.get_vehicle_state(self.uid)

    @property
    def fixed_delta_seconds(self):
        return self._fixed_delta_seconds

    def set_fixed_delta_seconds(self, val):
        self._fixed_delta_seconds = val

    def get_elapsed_seconds(self):
        return self.fixed_delta_seconds * self.frame

    def _update_clock(self):
        self._clock_writer.write(
            Clock(clock=round(self.get_elapsed_seconds() * 1e9)))

    def _update_chassis(self):
        vehicle_chassis = Chassis()
        vehicle_chassis.header.timestamp_sec = self.get_elapsed_seconds()
        vehicle_chassis.header.frame_id = 'ego_vehicle'
        vehicle_chassis.engine_started = True
        vehicle_chassis.throttle_percentage = 0.0
        vehicle_chassis.brake_percentage = 0.0
        vehicle_chassis.gear_location = self._gear_position
        if self._gear_position == vehicle_chassis.GEAR_REVERSE:
            vehicle_chassis.speed_mps = -self.state.v
        else:
            vehicle_chassis.speed_mps = self.state.v
        vehicle_chassis.driving_mode = Chassis.DrivingMode.COMPLETE_AUTO_DRIVE
        self._vehicle_chassis_writer.write(vehicle_chassis)

    def _update_localization(self):
        localization_estimate = LocalizationEstimate()
        localization_estimate.header.frame_id = 'novatel'
        localization_estimate.header.timestamp_sec = self.get_elapsed_seconds()

        vehicle_x, vehicle_y = self.state.vehicle_center
        localization_estimate.pose.position.x = vehicle_x
        localization_estimate.pose.position.y = vehicle_y
        localization_estimate.pose.position.z = 0

        w, x, y, z = self.state.orientation
        localization_estimate.pose.orientation.qw = w
        localization_estimate.pose.orientation.qx = x
        localization_estimate.pose.orientation.qy = y
        localization_estimate.pose.orientation.qz = z
        localization_estimate.pose.heading = self.state.theta_radians

        lvx, lvy = self.state.linear_velocity
        localization_estimate.pose.linear_velocity.x = lvx
        localization_estimate.pose.linear_velocity.y = lvy
        localization_estimate.pose.linear_velocity.z = 0

        av = self.state.angular_velocity
        localization_estimate.pose.angular_velocity.x = 0
        localization_estimate.pose.angular_velocity.y = 0
        localization_estimate.pose.angular_velocity.z = av

        avvx, avvy, avvz = transform_to_vrf(0, 0, av, self.state.theta_radians)
        localization_estimate.pose.angular_velocity_vrf.x = avvx
        localization_estimate.pose.angular_velocity_vrf.y = avvy
        localization_estimate.pose.angular_velocity_vrf.z = avvz

        lax, lay = self.state.linear_acceleration
        localization_estimate.pose.linear_acceleration.x = lax
        localization_estimate.pose.linear_acceleration.y = lay
        localization_estimate.pose.linear_acceleration.z = 0

        lavx, lavy, lavz = transform_to_vrf(
            lax, lay, 0, self.state.theta_radians)
        localization_estimate.pose.linear_acceleration_vrf.x = lavx
        localization_estimate.pose.linear_acceleration_vrf.y = lavy
        localization_estimate.pose.linear_acceleration_vrf.z = lavz
        self._vehicle_pose_writer.write(localization_estimate)

    def _get_obstacle_info_proto(self, uid):
        obj = PerceptionObstacle()
        obj.id = int(uid.split('-')[-1])

        obj_state = self.world_state.get_vehicle_state(uid)

        obj.position.x = obj_state.geometry_center.x
        obj.position.y = obj_state.geometry_center.y
        obj.position.z = 0

        obj.theta = obj_state.theta_radians

        vx, vy = obj_state.linear_velocity
        obj.velocity.x = vx
        obj.velocity.y = vy

        ax, ay = obj_state.linear_acceleration
        obj.acceleration.x = ax
        obj.acceleration.y = ay
        obj.acceleration.z = 0

        obj.length = obj_state.shape.length
        obj.width = obj_state.shape.width
        obj.height = obj_state.shape.height

        obj.type = PerceptionObstacle.VEHICLE

        return obj

    def _update_obstacles(self):
        obstacles = PerceptionObstacles()
        obstacles.header.timestamp_sec = self.get_elapsed_seconds()
        obstacles.header.frame_id = 'map'

        for uid in self.world_state.vehicle_ids:
            if uid != self.uid:
                obstacles.perception_obstacle.append(
                    self._get_obstacle_info_proto(uid))

        self._obstacles_writer.write(obstacles)

    def _update_traffic_light(self):
        traffic_light_detection = TrafficLightDetection()
        traffic_light_detection.header.timestamp_sec = self.get_elapsed_seconds()
        traffic_light_detection.header.frame_id = 'signal'

        for uid, state in self.world_state.traffic_lights.items():
            traffic_light = TrafficLight()
            traffic_light.id = uid
            if state == TrafficLightColor.UNKNOWN:
                traffic_light.color = traffic_light.UNKNOWN
            elif state == TrafficLightColor.GREEN:
                traffic_light.color = traffic_light.GREEN
            elif state == TrafficLightColor.RED:
                traffic_light.color = traffic_light.RED
            elif state == TrafficLightColor.YELLOW:
                traffic_light.color = traffic_light.YELLOW
            else:
                raise ValueError
            traffic_light_detection.traffic_light.add().CopyFrom(traffic_light)
        self._traffic_light_writer.write(traffic_light_detection)

    def update_input(self, perception=True):
        self._update_clock()
        self._update_chassis()
        self._update_localization()
        self._update_traffic_light()
        if perception:
            self._update_obstacles()

    def tick(self, world_state: WorldState, perception=True):
        self.set_world_state(world_state)
        self.update_input(perception)

    def on_trajectory_updated(self, data):
        if data.header.status.error_code == ErrorCode.PLANNING_ERROR:
            msg = f'Planning failed for {data.header.status.msg}'
            logger.warning(f'{msg}. (frame {self.frame})')
            self._on_state_updated(None, msg)
            return

        if data.estop.is_estop:
            msg = f'Planning produce estop for {data.estop.reason}'
            logger.warning(f'{msg} (frame {self.frame})')
            self._on_state_updated(None, msg)
            return

        trajectory_points = data.trajectory_point

        if len(trajectory_points) <= 1:
            msg = 'Planning is not ready'
            self._on_state_updated(None, msg)
            logger.warning(f'{msg} (frame {self.frame})')
            return

        trajectory_points = data.trajectory_point
        next_point_index = 0
        while next_point_index < len(trajectory_points) and \
                self.fixed_delta_seconds > trajectory_points[next_point_index].relative_time:
            next_point_index += 1

        if next_point_index >= len(trajectory_points):
            next_point_index = len(trajectory_points) - 1

        if next_point_index == 0:
            if self._prev_point is None:
                prev_point = vehicle_state_to_trajectory_point(self.state)
                logger.warning(f'Current time is smaller than the first trajectory point and there is no previous '
                               f'point. (frame {self.frame})')
            else:
                prev_point = self._prev_point
        else:
            prev_point_index = next_point_index - 1
            prev_point = trajectory_points[prev_point_index]

        next_point = trajectory_points[next_point_index]

        if self.fixed_delta_seconds > next_point.relative_time:
            point = next_point
        else:
            point = linear_interpolation(
                prev_point, next_point, self.fixed_delta_seconds)

        next_state = trajectory_point_to_vehicle_state(point, self.state.shape)
        self._on_state_updated(next_state, None)
        self._prev_point = point
