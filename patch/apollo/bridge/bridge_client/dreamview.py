from bridge_common.network_lib import Url
import websocket
import json
from bridge_common.vehicle_state import VehicleState
import time, os
import loguru
from abc import ABC


# MODE = 'Mkz Lgsvl'
MODE = 'Mkz Standard Debug'
VEHICLE = 'Lincoln2017MKZ LGSVL'
# VEHICLE = 'Mkz Example'
DEFAULT_MAP = 'Borregas Ave'


def _wait_content_in_file(file_path, content, timestep=0.1):
    is_ready = False
    while not is_ready:
        with open(file_path) as f:
            if content in f.read():
                is_ready = True
            else:
                time.sleep(timestep)


class DreamviewState:
    mode_name = None
    vehicle_name = None
    map_name = None

    def __str__(self):
        return f'(mode={self.mode_name}, vehicle_name={self.vehicle_name}, map_name={self.map_name})'


class Setter(ABC):
    def __init__(self, dreamview, target_name):
        self.dreamview = dreamview
        self.target_name = target_name

    def send_command(self):
        raise NotImplementedError

    def check_result(self):
        raise NotImplementedError

    def run(self):
        if not self.check_result():
            self.send_command()
            self.dreamview.update_state()
        while not self.check_result():
            self.dreamview.update_state()


class MapSetter(Setter):
    def send_command(self):
        self.dreamview.ws.send(json.dumps({
            'action': 'CHANGE_MAP',
            'type': 'HMIAction',
            'value': self.target_name
        }))
    
    def check_result(self):
        return self.dreamview.state.map_name == self.target_name


class VehicleSetter(Setter):
    def send_command(self):
        self.dreamview.ws.send(json.dumps({
            'action': 'CHANGE_VEHICLE',
            'type': 'HMIAction',
            'value': self.target_name
        }))

    def check_result(self):
        return self.dreamview.state.vehicle_name == self.target_name


class ModeSetter(Setter):
    def send_command(self):
        self.dreamview.ws.send(json.dumps({
            'action': 'CHANGE_MODE',
            'type': 'HMIAction',
            'value': self.target_name
        }))

    def check_result(self):
        return self.dreamview.state.mode_name == self.target_name
    

class Dreamview:
    def __init__(self, host, port):
        self.ws = websocket.create_connection(Url.websocket_url(host, port, 'websocket'))
        self.ws.settimeout(10)
        self.state = DreamviewState()
        self.init()

    def init(self):
        self.update_state()
        ModeSetter(self, MODE).run()
        VehicleSetter(self, VEHICLE).run()

    def reset(self):
        self.stop_modules()
        self.reset_backend()

    def update_state(self):
        while True:
            try:
                recv_data = json.loads(self.ws.recv())
                if 'data' in recv_data and 'currentMap' in recv_data['data']:
                    self.state.map_name = recv_data['data']['currentMap']
                    self.state.vehicle_name = recv_data['data']['currentVehicle']
                    self.state.mode_name = recv_data['data']['currentMode']
                    break
            except websocket.WebSocketTimeoutException:
                loguru.logger.error(f'Timeout when udpate state')
                break

    def stop_modules(self):
        self.ws.send(json.dumps({'type': 'HMIAction', 'action': 'RESET_MODE'}))

    def reset_backend(self):
        self.ws.send(json.dumps({'type': 'Reset'}))

    def start_modules(self, module_names):
        for module_name in module_names:
            self.start_module(module_name)
        self.wait_nodes_enabled([nid.lower() for nid in module_names])
        for module_name in module_names:
            self.wait_module_enabled(module_name)

    def start_module(self, module_name):
        self.ws.send(json.dumps({
            'action': 'START_MODULE',
            'type': 'HMIAction',
            'value': module_name
        }))

    def wait_nodes_enabled(self, node_names):
        is_ready = False
        while not is_ready:
            is_ready = True
            file_path = '/tmp/cyber_node_list_bridge.txt'
            os.system(f'cyber_node list > {file_path}')
            with open(file_path) as f:
                enabled_node_list = set(line.strip() for line in f.readlines())
            for node_name in node_names:
                if node_name not in enabled_node_list:
                    is_ready = False
                    break
            else:
                time.sleep(0.1)

    def wait_module_enabled(self, module_name):
        if module_name == 'Planning':
            time.sleep(0.5)  # TODO
        elif module_name == 'Routing':
            _wait_content_in_file('/apollo/data/log/routing.INFO', 'Routing service is ready')
        elif module_name == 'Prediction':
             _wait_content_in_file('/apollo/data/log/prediction.INFO', 'Defined default off lane obstacle predictor')
        loguru.logger.success(f'{module_name} is ready')

    def reload_tmp_map(self):
        MapSetter(self, DEFAULT_MAP).run()
        MapSetter(self, 'Tmp').run()

    def set_destination(self, vehicle_state: VehicleState, destination_x, destination_y):
        start_position = vehicle_state.vehicle_center - 5 * vehicle_state.forward_vector
        message = {
            'start': {'x': start_position.x, 'y': start_position.y, 'z': 0, 'heading': vehicle_state.theta_radians},
            'end': {'x': destination_x, 'y': destination_y, 'z': 0},
            'type': 'SendRoutingRequest',
            'waypoint': []
        }
        self.ws.send(json.dumps(message))

