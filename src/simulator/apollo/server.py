from websocket_server import WebsocketServer
from src.common.types import *
from src.simulator.apollo.message import Message
from src.simulator.apollo.simulator import ApolloSimulator, ApolloWorldModel, ApolloVehicleState, CarlaConfiguration
import json
import threading
import os
# from src.simulator.adapter. vista_util.vista_type import *
from loguru import logger


class ApolloSimulatorServer:
    def __init__(self, uid: str, port: int, initial_world_state: ApolloWorldModel, destinations: Mapping[str, Tuple[float, float]],
                 apollo_map_data: Optional[str], carla_configuration: CarlaConfiguration, fixed_delta_seconds: float = 0.1, callback: Optional[Callable] = None,
                 has_traffic_light=False,
                 other_config=None):

        # Check data consistency
        assert destinations.keys() == set(initial_world_state.autopilot_ids)

        self.uid = uid
        self.fixed_delta_seconds = fixed_delta_seconds

        self.apollo_map_data = apollo_map_data
        self.destinations = {
            vid: {'x': x, 'y': y} for vid, (x, y) in destinations.items()
        }

        self.simulator = ApolloSimulator(
            initial_world_state, carla_configuration, fixed_delta_seconds, has_traffic_light, other_config)

        self.callback = callback
        if self.callback is not None:
            self.callback(self.simulator, False)

        self.clients = dict()

        self.received_vehicles_states = dict()
        self.received_error_message = dict()

        self.response_lock = threading.Lock()
        self.init_lock = threading.Lock()
        self.ready_autopilots = set()
        self.server = WebsocketServer('0.0.0.0', port)

        self.server.set_fn_message_received(
            lambda client, server, message: self._on_message_received(client, server, message))
        self.server.set_fn_client_left(
            lambda client, server: self._on_client_left(client, server))

    def run(self):
        self.server.run_forever()

    def run_threading(self):
        server_thread = threading.Thread(target=self.run)
        server_thread.start()

    def send_message(self, client_uid, action, body):
        self.clients[client_uid]['handler'].send_message(json.dumps(
            Message(self.uid, action, self.simulator.frame, body=body).to_dict()))

    def _trigger_clients(self):
        logger.info(f'triggering clients (frame {self.simulator.frame})')
        self.received_vehicles_states.clear()
        self.received_error_message.clear()
        for uid in self.simulator.world_state.autopilot_ids:
            self.send_message(uid, 'TICK', {
                'world_state': self.simulator.get_partial_world_state(uid).to_dict(),
            })

    def _set_tmp_map(self, client, message: Message):
        if self.apollo_map_data:
            self.send_message(message.uid, 'LOAD_TMP_MAP', {
                'map_data': self.apollo_map_data
            })

    def _on_message_received(self, client, server, message):
        message = Message.from_dict(json.loads(message))
        if message.uid not in self.simulator.autopilot_ids:
            logger.warning(
                f'Message from vehicle {message.uid} received. But the vehicle is not in the vehicle list.')
            return
        with self.response_lock:
            if message.action == 'CONNECT':
                self._handle_connect(client, message)
            elif message.action == 'UPDATE_STATE':
                self._handle_update_state(message)
            elif message.action == 'INIT_FINISHED':
                self._handle_finished(message)
            else:
                raise ValueError(f'Unsupported action {message.action}')

    def _handle_finished(self, message: Message):
        uid = message.uid
        logger.success(f'{uid} is ready')
        self.ready_autopilots.add(uid)
        if len(self.ready_autopilots) == len(self.simulator.autopilot_ids):
            logger.success('All the autopilots are ready')
            self._trigger_clients()

    def _handle_connect(self, client, message: Message):
        uid = message.uid
        logger.info(f'{uid} connected')
        self.clients[uid] = client
        self.send_message(uid, 'SET_ACTOR', {
            'world_state': self.simulator.get_partial_world_state(uid).to_dict(),
            'destination': self.destinations[uid],
            'fixed_delta_seconds': self.fixed_delta_seconds,
            'map_data': self.apollo_map_data
        })

    def _handle_update_state(self, message: Message):
        if message.frame != self.simulator.frame:
            logger.warning(
                f'received message at frame {message.frame} (current frame {self.simulator.frame})')
            return

        logger.info(
            f'State from {message.uid} at frame {message.frame} received')
        if message.is_success:
            state = ApolloVehicleState.from_dict(message['vehicle_state'])
        else:
            state = None
            self.received_error_message[message.uid] = message['error_message']
        self.received_vehicles_states[message.uid] = state
        if len(self.received_vehicles_states) == len(self.simulator.autopilot_ids):
            # All vehicles' states received
            next_vehicles_states = dict()
            error_message = ''
            for uid in self.simulator.autopilot_ids:
                if uid in self.received_error_message:
                    error_message += f'{uid}: {self.received_error_message[uid]}\n'
            if error_message:
                logger.warning(f'Error reported \n{error_message}')
            for uid, v_state in self.received_vehicles_states.items():
                if v_state is None:
                    logger.warning(f'Vehicle {uid} returns None')
                    next_vehicles_states[uid] = self.simulator.vehicle_state(
                        uid)
                else:
                    next_vehicles_states[uid] = v_state
            self.simulator.tick(next_vehicles_states)
            if self.callback is not None:
                self.callback(self.simulator, error_message)
            self._trigger_clients()

    def _on_client_left(self, client, server):
        with self.response_lock:
            for uid, cli in self.clients.items():
                if cli['id'] == client['id']:
                    # raise ConnectionError(f'client for {uid} disconnected')
                    logger.warning(f'client for {uid} disconnected')
                    os._exit(1)
