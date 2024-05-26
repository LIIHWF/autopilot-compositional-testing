from common.types import *
from semantic_model.system_state.component import StaticScene, VehicleItineraryBase, VehicleDynamicState, \
    TrafficLightDynamicState, TrafficLightState, VehicleState, VehicleShape
from semantic_model.system_run.proto import system_run_pb2


class DynamicSettings(ProtoClass):
    @singledispatchmethod
    def __init__(self, vehicle_itinerary_bases: Mapping[str, VehicleItineraryBase]):
        self._vehicle_itinerary_bases = MappingProxyType(vehicle_itinerary_bases)

    @__init__.register
    def __init__proto(self, proto: system_run_pb2.DynamicSettings):
        self.__init__({
            vehicle_id: VehicleItineraryBase(itinerary_base_proto)
            for vehicle_id, itinerary_base_proto in proto.vehicle_itinerary_bases
        })

    def dump(self) -> system_run_pb2.DynamicSettings:
        proto = system_run_pb2.DynamicSettings()
        for vehicle_id, vehicle_itinerary_base in self.vehicle_itinerary_bases.items():
            proto.vehicle_itinerary_bases[vehicle_id] = vehicle_itinerary_base.dump()
        return proto

    @property
    def vehicle_itinerary_bases(self) -> Mapping[str, VehicleItineraryBase]:
        return self._vehicle_itinerary_bases

    def vehicle_itinerary_base(self, vehicle_id: str) -> VehicleItineraryBase:
        return self.vehicle_itinerary_bases[vehicle_id]


class DynamicSequences(ProtoClass):
    @singledispatchmethod
    def __init__(self,
                 vehicle_dynamic_state_sequences: Mapping[str, Tuple[VehicleDynamicState, ...]],
                 traffic_light_dynamic_state_sequences: Mapping[str, Tuple[TrafficLightDynamicState, ...]]):
        self._vehicle_dynamic_state_sequences = MappingProxyType(vehicle_dynamic_state_sequences)
        self._traffic_light_dynamic_state_sequences = MappingProxyType(traffic_light_dynamic_state_sequences)

    @__init__.register
    def __init__proto(self, proto: system_run_pb2.DynamicSequences):
        self.__init__(
            {
                vehicle_id: tuple(
                    VehicleDynamicState(vehicle_dynamic_state_proto)
                    for vehicle_dynamic_state_proto in vehicle_dynamic_state_sequence_proto
                ) for vehicle_id, vehicle_dynamic_state_sequence_proto in proto.vehicle_dynamic_state_sequences
            },
            {
                traffic_light_id: tuple(
                    TrafficLightDynamicState(traffic_light_dynamic_state_proto)
                    for traffic_light_dynamic_state_proto in traffic_light_dynamic_state_sequence_proto
                ) for traffic_light_id, traffic_light_dynamic_state_sequence_proto in proto.traffic_light_dynamic_state_sequences
            }
        )

    def dump(self) -> system_run_pb2.DynamicSequences:
        proto = system_run_pb2.DynamicSequences()
        for vehicle_id, vehicle_dynamic_state_sequence in self.vehicle_dynamic_state_sequences.items():
            proto.vehicle_dynamic_state_sequences[vehicle_id] = system_run_pb2.VehicleDynamicStateSequence()
            sequence_proto = proto.vehicle_dynamic_state_sequences[vehicle_id]
            for vehicle_dynamic_state in vehicle_dynamic_state_sequence:
                state_proto = sequence_proto.add()
                state_proto.CopyFrom(vehicle_dynamic_state.dump())

        for traffic_light_id, traffic_light_dynamic_state_sequence in self.traffic_light_dynamic_state_sequences.items():
            proto.traffic_light_dynamic_state_sequences[traffic_light_id] = system_run_pb2.TrafficLightDynamicStateSequence()
            sequence_proto = proto.traffic_light_dynamic_state_sequences[traffic_light_id]
            for traffic_light_dynamic_state in traffic_light_dynamic_state_sequence:
                state_proto = sequence_proto.add()
                state_proto.CopyFrom(traffic_light_dynamic_state.dump())
        return proto

    @property
    def vehicle_dynamic_state_sequences(self) -> Mapping[str, Tuple[VehicleDynamicState, ...]]:
        return self._vehicle_dynamic_state_sequences

    def vehicle_dynamic_state_sequence(self, vehicle_id: str) -> Tuple[VehicleDynamicState, ...]:
        return self.vehicle_dynamic_state_sequences[vehicle_id]

    def vehicle_dynamic_state(self, vehicle_id: str, tick: int) -> VehicleDynamicState:
        return self.vehicle_dynamic_state_sequence(vehicle_id)[tick]

    @property
    def traffic_light_dynamic_state_sequences(self) -> Mapping[str, Tuple[TrafficLightDynamicState, ...]]:
        return self._traffic_light_dynamic_state_sequences

    def traffic_light_state_sequence(self, traffic_light_id: str) -> Tuple[TrafficLightDynamicState, ...]:
        return self.traffic_light_dynamic_state_sequences[traffic_light_id]

    def traffic_light_dynamic_state(self, traffic_light_id: str, tick: int) -> TrafficLightDynamicState:
        return self.traffic_light_state_sequence(traffic_light_id)[tick]


class SystemRunHeader(ProtoClass):
    @singledispatchmethod
    def __init__(self, length: int, delta_t: int, vehicle_ids: Tuple[str, ...]):
        self._length = length
        self._delta_t = delta_t
        self._vehicle_ids = vehicle_ids

    @__init__.register
    def __init__proto(self, proto: system_run_pb2.SystemRunHeader):
        self.__init__(proto.length, proto.delta_t, tuple(proto.vehicle_ids))

    def dump(self) -> system_run_pb2.SystemRunHeader:
        proto = system_run_pb2.SystemRunHeader()
        proto.length = self.length
        proto.delta_t = self.delta_t
        for vehicle_id in self.vehicle_ids:
            proto.vehicle_ids.append(vehicle_id)
        return proto

    @property
    def length(self) -> int:
        return self._length

    @property
    def delta_t(self) -> Number:
        return self._delta_t

    @property
    def vehicle_ids(self) -> Tuple[str, ...]:
        return self._vehicle_ids


class DynamicState:
    def __init__(self, vehicle_dynamic_states: Mapping[str, VehicleDynamicState],
                 traffic_light_dynamic_states: Mapping[str, TrafficLightDynamicState]):
        self._vehicle_dynamic_states = MappingProxyType(vehicle_dynamic_states)
        self._traffic_light_dynamic_states = MappingProxyType(traffic_light_dynamic_states)

    @property
    def vehicle_dynamic_states(self) -> Mapping[str, VehicleDynamicState]:
        return self._vehicle_dynamic_states

    @property
    def traffic_light_dynamic_states(self) -> Mapping[str, TrafficLightDynamicState]:
        return self._traffic_light_dynamic_states


class Snapshot:
    def __init__(self, frame: int, time_seconds: Number, static_scene: StaticScene, dynamic_settings: DynamicSettings,
                 dynamic_state: DynamicState, vehicle_shape: VehicleShape):
        self._frame = frame
        self._time_seconds = time_seconds
        self._static_scene = static_scene
        self._dynamic_settings = dynamic_settings
        self._dynamic_state = dynamic_state
        self._vehicle_shape = vehicle_shape

    @property
    def frame(self) -> int:
        return self._frame

    def set_frame(self, frame: int):
        self._frame = frame

    @property
    def time_seconds(self) -> Number:
        return self._time_seconds

    @property
    def static_scene(self) -> StaticScene:
        return self._static_scene

    @property
    def dynamic_settings(self) -> DynamicSettings:
        return self._dynamic_settings

    @property
    def dynamic_state(self) -> DynamicState:
        return self._dynamic_state

    @cached_property
    def vehicle_ids(self) -> Tuple[str, ...]:
        return tuple(self._dynamic_settings.vehicle_itinerary_bases.keys())

    @property
    def vehicle_shape(self) -> VehicleShape:
        return self._vehicle_shape

    def vehicle_state(self, vehicle_id: str) -> VehicleState:
        return VehicleState(self.vehicle_shape,
                            self.dynamic_settings.vehicle_itinerary_bases[vehicle_id],
                            self.dynamic_state.vehicle_dynamic_states[vehicle_id])

    def traffic_light_state(self, traffic_light_id: str):
        return TrafficLightState(self.static_scene.static_object(traffic_light_id).state,
                                 self.dynamic_state.traffic_light_dynamic_states[traffic_light_id])


class SystemRun(ProtoClass):
    @singledispatchmethod
    def __init__(self, header: SystemRunHeader, static_scene: StaticScene,
                 dynamic_settings: DynamicSettings, dynamic_sequences: DynamicSequences):
        self._header = header
        self._static_scene = static_scene
        self._dynamic_settings = dynamic_settings
        self._dynamic_sequences = dynamic_sequences

    @__init__.register
    def __init__proto(self, proto: system_run_pb2.SystemRun):
        self.__init__(SystemRunHeader(proto.header), StaticScene(proto.static_scene),
                      DynamicSettings(proto.dynamic_settings), DynamicSequences(proto.dynamic_sequences))

    def dump(self) -> system_run_pb2.SystemRun:
        proto = system_run_pb2.SystemRun()
        proto.header.CopyFrom(self.header.dump())
        proto.static_scene.CopyFrom(self.static_scene.dump())
        proto.dynamic_settings.CopyFrom(self.dynamic_settings.dump())
        proto.dynamic_sequences.CopyFrom(self.dynamic_sequences.dump())
        return proto

    @property
    def header(self) -> SystemRunHeader:
        return self._header

    @property
    def static_scene(self) -> StaticScene:
        return self._static_scene

    @property
    def dynamic_settings(self) -> DynamicSettings:
        return self._dynamic_settings

    @property
    def dynamic_sequences(self) -> DynamicSequences:
        return self._dynamic_sequences

    def vehicle_state(self, vehicle_id: str, tick: int) -> VehicleState:
        return VehicleState(self.dynamic_settings.vehicle_itinerary_base(vehicle_id),
                            self.dynamic_sequences.vehicle_dynamic_state(vehicle_id, tick))

    def traffic_light_state(self, traffic_light_id: str, tick: int) -> TrafficLightState:
        return TrafficLightState(self.static_scene.static_object(traffic_light_id).state,
                                 self.dynamic_sequences.traffic_light_dynamic_state(traffic_light_id, tick))

    def snapshot(self, tick: int) -> Snapshot:
        return Snapshot(tick, tick * self.header.delta_t, self.static_scene, self.dynamic_settings,
                        DynamicState(
                            {
                                vid: state_sequence[tick] for vid, state_sequence in
                                self.dynamic_sequences.vehicle_dynamic_state_sequences.items()
                            },
                            {
                                tid: state_sequence[tick] for tid, state_sequence in
                                self.dynamic_sequences.traffic_light_dynamic_state_sequences.items()
                            }
                        ))
