import sys
sys.path.append('/apollo/bridge/')
import websocket
import json
import os
from bridge_client.vehicle import Vehicle
import threading
from loguru import logger
import traceback
import netifaces
import signal

from typing import Optional

from bridge_common.vehicle_state import VehicleState
from bridge_common.world_model import WorldState
from bridge_common.network_lib import Message, Url
from bridge_client.dreamview import Dreamview
from cyber_compatibility.node import CompatibleNode

# Protobuf
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis
from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate, LocalizationStatus
from modules.common_msgs.planning_msgs.planning_pb2 import ADCTrajectory
from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import PerceptionObstacles
from cyber.proto.clock_pb2 import Clock
from modules.common_msgs.transform_msgs.transform_pb2 import TransformStampeds
from modules.common_msgs.perception_msgs.traffic_light_detection_pb2 import TrafficLightDetection


def print_error(error, skip_exception=True):
    if isinstance(error, KeyboardInterrupt):
        os._exit(0)
    if skip_exception and type(error) in [websocket.WebSocketConnectionClosedException, ConnectionRefusedError]:
        raise type(error)()
    print(traceback.format_exc())


def create_node(name='cyber_bridge_node'):
    node_ = CompatibleNode(name)
    channels_ = dict()

    channels_['vehicle_chassis_writer'] = node_.new_writer(
        "/apollo/canbus/chassis",
        Chassis,
        qos_depth=10)
    channels_['vehicle_pose_writer'] = node_.new_writer(
        "/apollo/localization/pose",
        LocalizationEstimate,
        qos_depth=10)
    channels_['localization_status_writer'] = node_.new_writer(
        "/apollo/localization/msf_status",
        LocalizationStatus,
        qos_depth=10)
    channels_['tf_writer'] = node_.new_writer("/tf", TransformStampeds)
    channels_['obstacles_writer'] = node_.new_writer(
        "/apollo/perception/obstacles",
        PerceptionObstacles,
        qos_depth=10)
    channels_['clock_writer'] = node_.new_writer('/clock', Clock, 10)
    channels_['trajectory_reader'] = node_.new_reader(
        "/apollo/planning",
        ADCTrajectory,
        lambda data: client.vehicle.on_trajectory_updated(data)
    )
    channels_['traffic_light_writer'] = node_.new_writer(
        "/apollo/perception/traffic_light",
        TrafficLightDetection,
        qos_depth=10)
    return node_, channels_


node, channels = create_node()

def get_host_ip():
    for interface in netifaces.interfaces():
        for link in netifaces.ifaddresses(interface).get(netifaces.AF_INET, []):
            if link['addr'] == '127.0.0.1':
                continue
            parts = link['addr'].split('.')
            parts[-1] = '1'
            return '.'.join(parts)
    return None
            

class BridgeClient:
    def __init__(self, settings):
        self.response_lock = threading.Lock()

        self.vehicle_uid = settings["vehicle"]["uid"]
        
        if settings_dict['bridge_server']['host'] == 'auto':
            self.host = get_host_ip()
        else:
            self.host = settings_dict['bridge_server']['host']
        
        self.ws = websocket.WebSocketApp(
            Url.websocket_url(self.host, settings_dict['bridge_server']['port']),
            on_message=lambda ws, message: self._on_message(ws, message),
            on_open=lambda ws: self._on_open(),
            on_error=lambda ws, error: print_error(error))

        self.dreamview = Dreamview('localhost', 8888)
        self.dreamview.reset()

        self.vehicle = Vehicle(
            settings['vehicle']['uid'], channels, self._on_state_updated)
        self.initialized = False

    def run(self):
        self.ws.run_forever()

    def destroy(self):
        self.ws.close()

    def send_message(self, action: str, status=True, body=None):
        self.ws.send(json.dumps(
            Message(self.vehicle.uid, action, self.vehicle.frame, status, body).to_dict()))

    def _on_open(self):
        logger.info('Server connected')
        self.send_message('CONNECT')

    def _on_message(self, ws, message):
        message = Message(json.loads(message))
        with self.response_lock:
            if message.action == 'SET_ACTOR':
                self._handle_set_actor(message)
            elif message.action == 'TICK':
                self._handle_tick(message)

    def _handle_load_tmp_map(self, map_data):
        os.system('rm /apollo/modules/map/data/tmp/*')
        with open('/apollo/modules/map/data/tmp/base_map.txt', 'w') as f:
            f.write(map_data)
        os.system('/apollo/bridge/scripts/import_map.sh tmp')
        self.dreamview.reload_tmp_map()

    def _handle_set_actor(self, message: Message):
        self.initialized = False

        self.vehicle.set_fixed_delta_seconds(message['fixed_delta_seconds'])
        self._handle_tick(message, False)  # need message['world_state']

        if message['map_data'] != 'null':
            self._handle_load_tmp_map(message['map_data'])

        self.dreamview.start_modules(['Routing', 'Prediction', 'Planning'])
        self.dreamview.set_destination(self.vehicle.state,
                                       message['destination']['x'],
                                       message['destination']['y'])
        self.initialized = True
        self.send_message('INIT_FINISHED')

    def _on_state_updated(self, state: Optional[VehicleState], error_message: Optional[str]):
        if not self.initialized:
            logger.error(f'Update state before initialization')
            return
        if state is None:
            self.send_message('UPDATE_STATE', False, body={
                'error_message': error_message
            })
        else:
            self.send_message('UPDATE_STATE', body={
                'vehicle_state': state.to_dict()
            })

    def _handle_tick(self, message, perception=True):
        self.vehicle.tick(WorldState(message['world_state']), perception)


def quit(signo, _frame):
    print("\nInterrupted by %d, shutting down" % signo)
    os._exit(0)


if __name__ == '__main__':
    config_file = "/apollo/bridge_config/settings.json"
    settings_dict = json.load(open(config_file))

    for sig in ('TERM', 'HUP', 'INT'):
        signal.signal(getattr(signal, 'SIG'+sig), quit)

    try:
        client = BridgeClient(settings_dict)
        client.run()
    except websocket.WebSocketConnectionClosedException:
        logger.warning('Connection was lost. Retrying...')
    except ConnectionRefusedError:
        logger.warning(f'Connection refused. Waiting for the server on {client.host}...')
    except BrokenPipeError:
        logger.warning('Pipe broken. Retrying...')
    except KeyboardInterrupt:
        print('keyboard interrupt')
