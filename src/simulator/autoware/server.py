from websocket_server import WebsocketServer
from src.common.types import *
from src.simulator.autoware.message import Message
from src.simulator.autoware.autoware_simulator import AutowareSimulator
from src.simulator.autoware.autoware_world_model import AutowareWorldModel, AutowareVehicleState
import json
import threading
from loguru import logger
import os


class AutowareSimulatorServer:
    def __init__(self, uid, port, initial_world_state: AutowareWorldModel, destinations, map_data, collision_detected_actor=None, callback=None,
                 enable_carla=True, config=None):
        self.uid = uid
        
        self.clients = dict()
        self.destinations = destinations
        self.map_data = map_data

        self.simulator = AutowareSimulator(
            initial_world_state, map_data, 80, collision_detected_actor, enable_carla, config)
        assert set(self.destinations.keys()) == set(
            self.simulator.autoware_world_state.autopilot_ids)

        self.received_vehicles_states = dict()
        self.received_error_message = dict()

        self.response_lock = threading.Lock()
        self.init_lock = threading.Lock()
        self.ready_autopilots = set()

        self.server = WebsocketServer('0.0.0.0', port)

        self.callback = callback

        self.server.set_fn_message_received(
            lambda client, server, message: self._on_message_received(client, server, message))
        self.server.set_fn_client_left(
            lambda client, server: self._on_client_left(client, server))

    @property
    def world_state(self):
        return self.simulator.autoware_world_state

    @property
    def autopilot_ids(self):
        return self.simulator.autoware_world_state.autopilot_ids

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
        # partial_world_states = self.simulator.get_partial_world_states()
        for uid in self.simulator.autoware_world_state.autopilot_ids:
            self.send_message(uid, 'TICK', {
                'world_state': self.simulator.get_vista_for_vehicle(uid).to_dict(),
            })

    def _set_tmp_map(self, client, message: Message):
        raise NotImplementedError  # TODO
        if self.map_pattern is None:
            return
        self.send_message(message.uid, 'LOAD_TMP_MAP', {
            'map_data': 'TODO'
        })

    def _on_message_received(self, client, server, message):
        message = Message(json.loads(message))
        # if message.uid not in self.autopilot_ids:
        #     logger.warning(
        #         f'Message from vehicle {message.uid} received. But the vehicle is not in the autopilot list.')
        #     return
        with self.response_lock:
            if message.action == 'CONNECT':
                self._handle_connect(client, message)
            elif message.action == 'UPDATE_STATE':
                self._handle_update_state(message)
            elif message.action == 'INIT_FINISHED':
                self._handle_init_finished(message)
            else:
                raise ValueError(f'Unsupported action {message.action}')

    def _handle_init_finished(self, message: Message):
        uid = message.uid
        logger.success(f'{uid} is ready')
        self.ready_autopilots.add(uid)
        if len(self.ready_autopilots) == len(self.autopilot_ids):
            logger.success('All the autopilots are ready')
            self._trigger_clients()

    def _handle_connect(self, client, message: Message):
        message_uid = message.uid
        logger.info(f'{message_uid} connected')
        if message_uid in self.autopilot_ids and message_uid not in self.clients:
            self.clients[message_uid] = client
            new_uid = message_uid
        else:
            new_uid = None
            for uid in self.autopilot_ids:
                if uid not in self.clients:
                    self.clients[uid] = client
                    new_uid = uid
                    break
            else:
                logger.warning(f'Quiting {message_uid}')
                # client['handler'].send_message(json.dumps(
                #    Message(self.uid, 'QUIT', self.simulator.frame, body={}).to_dict()))
                client['handler'].request.close()
                return
        self.send_message(new_uid, 'SET_ACTOR', {
            'uid': new_uid,
            'world_state': self.simulator.get_vista_for_vehicle(new_uid).to_dict(),
            'destination': self.destinations[new_uid]
            # 'map_data': 'null' if self.map_pattern is None else self.map_pattern.apollo_map
        })

    def _handle_update_state(self, message: Message):
        if message.frame != self.simulator.frame:
            logger.warning( 
                f'received message at frame {message.frame} with action {message.action} (current frame {self.simulator.frame})')
            return

        logger.info(
            f'State from {message.uid} at frame {message.frame} received')
        if message.is_success:
            state = AutowareVehicleState(message['vehicle_state'])
        else:
            state = None
            self.received_error_message[message.uid] = message['error_message']
        self.received_vehicles_states[message.uid] = state
        if len(self.received_vehicles_states) == len(self.autopilot_ids):
            # All vehicles' states received
            next_vehicles_states = dict()
            error_message = ''
            for uid in self.autopilot_ids:
                if uid in self.received_error_message:
                    error_message += f'{uid}: {self.received_error_message[uid]}\n'
            if error_message:
                logger.warning(f'Error reported \n{error_message}')
            for uid, v_state in self.received_vehicles_states.items():
                if v_state is None:
                    logger.warning(f'Vehicle {uid} returns None')
                    next_vehicles_states[uid] = self.world_state.get_vehicle_state(
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
                    logger.error(f'client for {uid} disconnected')
                    os._exit(0)
                    # raise ConnectiofprnError(f'client for {uid} disconnected')
