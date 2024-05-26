import sys
sys.path.append('./')
import rclpy
import loguru
import json
import websocket
from client.autoware_vehicle_state import AutowareVehicleState, AutowareVehicleShape
from client.autoware_world_state import AutowareWorldModel
from client.message import Message, Url
from client.vehicle import Vehicle
import threading
from loguru import logger
import netifaces
import traceback
import time
import os
import signal
from typing import Optional


home = os.path.expanduser("~")

def print_error(error, skip_exception=True):
    if isinstance(error, KeyboardInterrupt):
        os._exit(0)
    if skip_exception and type(error) in [websocket.WebSocketConnectionClosedException, ConnectionRefusedError]:
        raise type(error)()
    print(traceback.format_exc())


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
        self.frame = 0

        if settings_dict['bridge_server']['host'] == 'auto':
            self.host = get_host_ip()
        else:
            self.host = settings_dict['bridge_server']['host']

        self.ws = websocket.WebSocketApp(
            Url.websocket_url(
                self.host,
                settings_dict['bridge_server']['port']),
            on_message=lambda ws, message: self._on_message(ws, message),
            on_open=lambda ws: self._on_open(),
            on_error=self._on_error
            )
        self.vehicle = None
        self.settings = settings
        self.initialized = False

    def _on_error(self, ws, error):
        if 'Connection refused' in str(error):
            logger.warning(f'Waiting for the server on {self.host}')
        else:
            logger.warning(f'{error}')
            print_error(error)
        exit()

    def run(self):
        self.ws.run_forever()

    def destroy(self):
        self.ws.close()

    def send_message(self, action: str, status=True, body=None):
        self.ws.send(json.dumps(
            Message(self.vehicle_uid, action, self.frame, status, body).to_dict()))

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
            elif message.action == 'QUIT':
                exit()

    def _handle_set_actor(self, message: Message):
        self.vehicle_uid = message['uid']
        self.vehicle = Vehicle(self.vehicle_uid, self._on_state_updated)

        self.initialized = False
        self.vehicle.init_vehicle(AutowareWorldModel(message['world_state']),
                                  (message['destination']['x'], message['destination']['y'], message['destination']['theta']))
        self.initialized = True
        self.send_message('INIT_FINISHED')

    def _on_state_updated(self, state: Optional[AutowareVehicleState], error_message: Optional[str]):
        logger.info(f'on_state_update: {state}')
        if not self.initialized:
            logger.warning(f'Update state before initialization')
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
        loguru.logger.info('handling tick')
        assert self.vehicle is not None
        self.vehicle.on_new_world_state(
            AutowareWorldModel(message['world_state']))
        self.vehicle.tick()
        self.frame += 1


def quit(signo, _frame):
    print("\nInterrupted by %d, shutting down" % signo)
    os._exit(0)


def read_status():
    with open(os.path.join(home, "config/dashboard_status.txt"), "r") as file:
        return file.read().strip()


def write_status(status: str):
    with open(os.path.join(home, "config/dashboard_status.txt"), "w") as file:
        file.write(status)


if __name__ == '__main__':
    config_file = os.path.join(home, "config/settings.json")
    settings_dict = json.load(open(config_file))

    for sig in ('TERM', 'HUP', 'INT'):
        signal.signal(getattr(signal, 'SIG'+sig), quit)

    rclpy.init()

    client = None
    if read_status() != 'started':
        logger.warning('Dashboard is not started. Waiting...')
        exit()
    try:
        client = BridgeClient(settings_dict)
        client.run()
    except websocket.WebSocketConnectionClosedException as e:
        logger.warning(f'Connection was lost for {e}. Retrying...')
        print(traceback.format_exc())
    except ConnectionRefusedError:
        logger.warning('Connection refused. Retrying...')
    except BrokenPipeError:
        logger.warning('Pipe broken. Retrying...')
    except KeyboardInterrupt:
        print('keyboard interrupt')
    finally:
        rclpy.shutdown()
        if client is not None and read_status() != 'started':
            write_status('finished')
