import math
import time
from loguru import logger
import uuid
import itertools
import os
from typing import Optional, Mapping, Any
from rclpy.executors import SingleThreadedExecutor
from client.autoware_vehicle_state import AutowareVehicleState, AutowareVehicleShape
from client.autoware_world_state import AutowareWorldModel, TrafficLightColor
from typing import Tuple
from autoware_perception_msgs.msg import TrafficSignalArray, TrafficSignal, TrafficSignalElement
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import rclpy
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_adapi_v1_msgs.srv import InitializeLocalization, SetRoutePoints, ChangeOperationMode
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState, OperationModeState, RouteState
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, TwistStamped, AccelWithCovarianceStamped
from autoware_auto_vehicle_msgs.msg import VelocityReport
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from dummy_perception_publisher.msg import Object, InitialState
from unique_identifier_msgs.msg import UUID
from autoware_adapi_v1_msgs.msg import ObjectClassification
from autoware_auto_perception_msgs.msg import Shape


VEHICLE_LENGTH = 4.72
VEHICLE_WIDTH = 2.089
VEHICLE_HEIGHT = 1.441
BACK_TO_CENTER = 0.977
FRONT_TO_CENTER = 4.72 - 0.977

LOG_CONTENT = ''


COV36 = \
    [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.001]

home = os.path.expanduser("~")

def write_status(status: str):
    with open(os.path.join(home, "config/dashboard_status.txt"), "w") as file:
        file.write(status)

class AutowareState:
    def __init__(self):
        self.localization_initialzied = False
        self.has_set_route = False
        self.enabled_control = False
        self.enabled_auto_mode = False

    def localization_initialzied_valid(self):
        self.localization_initialzied = True

    def has_set_route_valid(self):
        self.has_set_route = True

    def enabled_auto_mode_valid(self):
        self.enabled_auto_mode = True

    def enabled_control_valid(self):
        self.enabled_control = True


def xytheta_to_pose_msg(x, y, theta, with_covariance=False):
    if with_covariance:
        ret = PoseWithCovariance()
        pose = ret.pose
    else:
        ret = Pose()
        pose = ret
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0
    pose.orientation.w = math.cos(theta / 2)
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(theta / 2)
    if isinstance(ret, PoseWithCovariance):
        ret.covariance = COV36
    return ret


def generate_random_uuid_array():
    random_uuid = uuid.uuid4()
    uuid_bytes = random_uuid.bytes
    uuid_array = [byte for byte in uuid_bytes]
    return uuid_array


def time_msg_to_float(msg: Time):
    return msg.sec + msg.nanosec * 1e-9


class Vehicle:
    def __init__(self, uid: str, on_state_updated):
        self.uid = uid
        self.shape = AutowareVehicleShape({
            'front_edge_to_center': FRONT_TO_CENTER,
            'back_edge_to_center': BACK_TO_CENTER,
            'left_edge_to_center': VEHICLE_WIDTH / 2,
            'right_edge_to_center': VEHICLE_WIDTH / 2,
            'height': VEHICLE_HEIGHT
        }
        )
        self.on_state_updated = on_state_updated
        self.node = rclpy.create_node('scenario_runner', parameter_overrides=[
            rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.sim_time = 0  # read_sim_time()
        self.prev_sim_time = None
        self.autoware_state = AutowareState()
        
        self.latest_kinematic_state_time = 0.0
        self.latest_acceleration_time = 0.0
        
        self.initialized = False
        
        self.interactive_tick = False

        self.speed = 0
        self.kinematic_state = None
        self.acceleration = None
        self.localization_state = None

        self.has_callback = False

        self.dummpy_vehicle_messages = dict()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            # durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL,
            depth=10
        )

        self.init_localization_client = self.node.create_client(
            InitializeLocalization, '/api/localization/initialize')
        self.set_route_client = self.node.create_client(
            SetRoutePoints, '/planning/mission_planning/set_route_points')
        self.auto_mode_client = self.node.create_client(
            ChangeOperationMode, '/api/operation_mode/change_to_autonomous')

        self.localization_state_subscription = self.node.create_subscription(
            LocalizationInitializationState, '/api/localization/initialization_state',
            self._on_localization_state, qos_profile)

        self.operation_mode_subscription = self.node.create_subscription(
            OperationModeState, '/api/operation_mode/state',
            self._on_operation_mode, qos_profile)

        self.route_state_subscription = self.node.create_subscription(
            RouteState, '/api/routing/state',
            self._on_route_state, qos_profile)

        self.velocity_report_subscription = self.node.create_subscription(
            VelocityReport, '/vehicle/status/velocity_status',
            self._on_velocity_report, 10
        )

        self.kinematic_state_subcription = self.node.create_subscription(
            Odometry, '/localization/kinematic_state',
            self._on_kinematic_state, qos_profile
        )

        self.acceleration_subscription = self.node.create_subscription(
            AccelWithCovarianceStamped, 'localization/acceleration',
            self._on_acceleration, qos_profile
        )

        self.latest_control_cmd_time = 0.0
        self.control_cmd_subscription = self.node.create_subscription(
            AckermannControlCommand, '/control/command/control_cmd',
            self._on_control_cmd, qos_profile)

        self.clock_publisher = self.node.create_publisher(
            Clock, '/clock', qos_profile)
        self.initialtwist_publisher = self.node.create_publisher(
            TwistStamped, '/simulation/input/initialtwist', qos_profile)
        # self.kinematic_state_publisher = self.node.create_publisher(
        #     Odometry, '/localization/kinematic_state', qos_profile)
        # self.velocity_report_publisher = self.node.create_publisher(
        #     VelocityReport, '/vehicle/status/velocity_status', 10)
        self.dummy_perception_publisher = self.node.create_publisher(
            Object, '/simulation/dummy_perception_publisher/object_info', qos_profile)
        self.traffic_light_publisher = self.node.create_publisher(
            TrafficSignalArray, '/perception/traffic_light_recognition/traffic_signals', 10)

    def _on_control_cmd(self, msg: AckermannControlCommand):
        self.latest_control_cmd_time = self.sim_time
        self.has_callback = True

    def _on_kinematic_state(self, msg: Odometry):
        self.latest_kinematic_state_time = time_msg_to_float(msg.header.stamp)
        self.has_callback = True

        self.kinematic_state = {
            'time': self.sim_time,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) * 2,
            'speed': msg.twist.twist.linear.x
        }
        self._try_on_state_updated()

    def _on_acceleration(self, msg: AccelWithCovarianceStamped):
        self.latest_acceleration_time = time_msg_to_float(msg.header.stamp)
        self.has_callback = True

        self.acceleration = msg.accel.accel.linear.x
        self._try_on_state_updated()

    def _try_on_state_updated(self):
        if self.kinematic_state is not None and self.acceleration is not None:
            kinematic_state = self.kinematic_state
            acceleration = self.acceleration
            self.kinematic_state = None
            self.acceleration = None
            # input(
            #     f"sending kinematic state {(kinematic_state['x'], kinematic_state['y'], kinematic_state['speed'], acceleration, kinematic_state['theta'], self.shape)}")
            
            if self.initialized:
                self.on_state_updated(AutowareVehicleState(
                    kinematic_state['x'], kinematic_state['y'], kinematic_state['speed'], acceleration, kinematic_state['theta'],
                    self.shape
                ), None)

    def set_dummy_vehicle(self, name, x, y, theta, speed):
        msg = Object()
        msg.header.stamp = self.sim_time_msg()
        msg.header.frame_id = 'map'

        # make x, y be the center of the vehicle, where the given x, y is considering the center of the rear axle
        x = x + (VEHICLE_LENGTH / 2 - BACK_TO_CENTER) * math.cos(theta)
        y = y + (VEHICLE_LENGTH / 2 - BACK_TO_CENTER) * math.sin(theta)

        if name in self.dummpy_vehicle_messages:
            msg.id = self.dummpy_vehicle_messages[name].id
        else:
            msg.id.uuid = generate_random_uuid_array()
        msg.initial_state = InitialState()
        msg.initial_state.pose_covariance = xytheta_to_pose_msg(
            x, y, theta, with_covariance=True)
        msg.max_velocity = 33.3
        msg.min_velocity = 0.0
        msg.initial_state.twist_covariance.twist.linear.x = speed
        msg.initial_state.twist_covariance.covariance = COV36
        msg.initial_state.accel_covariance.covariance = COV36

        msg.classification.label = ObjectClassification.CAR
        msg.classification.probability = 1.0

        msg.shape.type = Shape.BOUNDING_BOX
        msg.shape.dimensions.x = VEHICLE_LENGTH
        msg.shape.dimensions.y = VEHICLE_WIDTH
        msg.shape.dimensions.z = VEHICLE_HEIGHT

        if name in self.dummpy_vehicle_messages:
            msg.action = Object.MODIFY
        else:
            msg.action = Object.ADD

        self.dummpy_vehicle_messages[name] = msg
        self.dummy_perception_publisher.publish(msg)

    def _on_velocity_report(self, msg):
        self.has_callback = True
        self.speed = msg.longitudinal_velocity

    def _on_route_state(self, msg):
        self.has_callback = True
        if msg.state == RouteState.SET:
            self.autoware_state.has_set_route_valid()
        if msg.state == RouteState.ARRIVED:
            logger.success('Arrived')
            # raise InterruptedError('Arrived')
        else:
            self.node.get_logger().info(f'[on_route_state] {msg}')

    def _on_localization_state(self, msg):
        self.has_callback = True
        if msg.state == LocalizationInitializationState.UNINITIALIZED:
            self.localization_state = LocalizationInitializationState.UNINITIALIZED
        if msg.state == LocalizationInitializationState.INITIALIZED:
            self.autoware_state.localization_initialzied_valid()
        else:
            self.node.get_logger().info(f'[on_localization_state] {msg}')

    def _on_operation_mode(self, msg):
        self.has_callback = True
        if msg.mode == OperationModeState.AUTONOMOUS:
            self.autoware_state.enabled_auto_mode_valid()
        if msg.is_autoware_control_enabled:
            self.autoware_state.enabled_control_valid()

    def sim_time_msg(self):
        msg = Time()
        msg.sec = int(self.sim_time)
        msg.nanosec = int((self.sim_time - int(self.sim_time)) * 1e9)
        return msg

    def publish_clock(self):
        msg = Clock()
        msg.clock = self.sim_time_msg()
        self.clock_publisher.publish(msg)

    def set_init_pose(self, x, y, theta):
        req = InitializeLocalization.Request()
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.sim_time_msg()
        pose.header.frame_id = 'map'
        pose.pose = xytheta_to_pose_msg(x, y, theta, with_covariance=True)
        req.pose = [pose]
        future = self.init_localization_client.call_async(req)
        return future

    def set_destination(self, x, y, theta):
        x = float(x)
        y = float(y)
        theta = float(theta)
        req = SetRoutePoints.Request()
        req.header.stamp = self.sim_time_msg()
        req.header.frame_id = 'map'
        req.option.allow_goal_modification = False
        req.goal = xytheta_to_pose_msg(x, y, theta)
        future = self.set_route_client.call_async(req)
        return future

    def set_auto_mode(self):
        req = ChangeOperationMode.Request()
        future = self.auto_mode_client.call_async(req)
        return future

    def update_sim_time_for_one_tick(self):
        # self.sim_time = round(self.sim_time + 0.1, 1)
        # self.sim_time += 0.00001
        self.sim_time = self.sim_time + 0.100001

    def tick(self, times=1, gap=0.2):
        for _ in range(times):
            if self.sim_time > 0.3:
                for i in range(50):
                    if self.sim_time - self.latest_control_cmd_time < 0.05:
                        break
                    self.executor.spin_once(gap)
                else:
                    # input('press enter to exit')
                    raise InterruptedError('Failed to get control_cmd')
            self.update_sim_time_for_one_tick()
            self.publish_clock()
            self.executor.spin_once()

            self.has_callback = True
            for i in range(50):
                if self.sim_time - self.latest_acceleration_time < 0.05 and self.sim_time - self.latest_kinematic_state_time < 0.05:
                    break
                self.executor.spin_once(gap)
            else:
                raise InterruptedError('Failed to get vehicle state')
            
            if self.sim_time < 0.35 and self.sim_time > 0.3:
                for i in range(50):
                    if self.autoware_state.has_set_route:
                        break
                    self.executor.spin_once(gap)
                else:
                    raise InterruptedError('Failed to set route')
            # input('continue')
    
    def publish_traffic_light(self, tid, color):
        msg = TrafficSignalArray()
        msg.stamp = self.sim_time_msg()
        msg.signals = [TrafficSignal()]
        msg.signals[0].traffic_signal_id = int(tid)  # 1377
        msg.signals[0].elements = [TrafficSignalElement()]
        if color == 'red':
            msg.signals[0].elements[0].color = TrafficSignalElement.RED
        elif color == 'green':
            msg.signals[0].elements[0].color = TrafficSignalElement.GREEN
        elif color == 'yellow':
            msg.signals[0].elements[0].color = TrafficSignalElement.AMBER

        msg.signals[0].elements[0].shape = TrafficSignalElement.CIRCLE
        msg.signals[0].elements[0].status = TrafficSignalElement.SOLID_ON
        msg.signals[0].elements[0].confidence = 1.0
        self.traffic_light_color = color
        self.traffic_light_publisher.publish(msg)
    
    def init_vehicle(self, world_state: AutowareWorldModel, destination: Tuple[float, float, float]):
        time.sleep(1)
        ego_state = world_state.get_vehicle_state(self.uid)
        
        future = self.set_init_pose(ego_state.x, ego_state.y, ego_state.theta_radians)
        # for vehicle_id, vehicle_state in world_state.vehicle_states.items():
        #         if vehicle_id != self.uid:
        #             self.set_dummy_vehicle(vehicle_id, vehicle_state.x, vehicle_state.y,
        #                                     vehicle_state.theta_radians, vehicle_state.v)
        # input('init')
        
        for tid, color in world_state.traffic_lights.items():
            self.publish_traffic_light(tid, color.name.lower())
        
        write_status('initialized')
        for i in range(2):
            time.sleep(0.5)
            self.tick(1, 0.2)
            self.set_init_speed(ego_state.v)
        self.executor.spin_until_future_complete(future)
        
        waiting_tick = 3
        for vehicle_id, vehicle_state in world_state.vehicle_states.items():
            if vehicle_id != self.uid:
                self.set_dummy_vehicle(vehicle_id, 
                                        vehicle_state.x - math.cos(vehicle_state.theta_radians) * waiting_tick * 0.1 * vehicle_state.v,
                                        vehicle_state.y - math.sin(vehicle_state.theta_radians) * waiting_tick * 0.1 * vehicle_state.v,
                                        vehicle_state.theta_radians, vehicle_state.v)
        
        
        destination_future = self.set_destination(destination[0], destination[1], destination[2])
        auto_future = self.set_auto_mode()
        time.sleep(1)
        self.tick(1, 0.2)
        while self.autoware_state.has_set_route == False:
            self.executor.spin_once(0.1)
        self.executor.spin_until_future_complete(destination_future)
        # self.tick(1, 0.2)
        self.executor.spin_until_future_complete(auto_future)
        
        logger.info('Set destination and auto mode successfully')

        for i in range(waiting_tick - 2):
            self.tick(1, 0.5)
            self.set_init_speed(ego_state.v)    

        # for i in range(waiting_tick):
        #     self.tick()
        #     self.set_init_speed(ego_state.v)
            

        self.initialized = True


    def set_init_speed(self, speed):
        msg = TwistStamped()
        msg.header.stamp = self.sim_time_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = speed
        self.initialtwist_publisher.publish(msg)

    def on_new_world_state(self, world_state: AutowareWorldModel):
        for vehicle_id, vehicle_state in world_state.vehicle_states.items():
            if vehicle_id != self.uid:
                self.set_dummy_vehicle(vehicle_id, vehicle_state.x, vehicle_state.y,
                                       vehicle_state.theta_radians, vehicle_state.v)
            
        for tid, color in world_state.traffic_lights.items():
            self.publish_traffic_light(tid, color.name.lower())