from src.common.types import *
from src.semantic_model.geometry import *
from src.semantic_model.system_state.component.static import Position
from src.semantic_model.system_state.proto import dynamic_state_pb2


class VehicleShape(ProtoClass):
    @singledispatchmethod
    def __init__(self, length: Number, width: Number):
        self._length = length
        self._width = width

    @__init__.register
    def __init__proto(self, proto: dynamic_state_pb2.VehicleShape):
        self.__init__(proto.length, proto.width)

    def dump(self) -> dynamic_state_pb2.VehicleShape:
        proto = dynamic_state_pb2.VehicleShape()
        proto.length = self.length
        proto.width = self.width
        return proto

    @property
    def length(self) -> Number:
        return self._length

    @property
    def width(self) -> Number:
        return self._width


class VehicleItineraryBase(ProtoClass):
    @singledispatchmethod
    def __init__(self, start_offset: Number, edge_id_sequence: Iterable[str], end_offset: Number):
        self._start_offset = start_offset
        self._edge_id_sequence = tuple(edge_id_sequence)
        self._end_offset = end_offset

    @__init__.register
    def __init__proto(self, proto: dynamic_state_pb2.VehicleItineraryBase):
        self.__init__(proto.start_offset, proto.edge_id_sequence, proto.end_offset)

    def dump(self) -> dynamic_state_pb2.VehicleItineraryBase:
        proto = dynamic_state_pb2.VehicleItineraryBase()
        proto.start_offset = self.start_offset
        proto.end_offset = self.end_offset
        for edge_id in self.edge_id_sequence:
            proto.edge_id_sequence.append(edge_id)
        return proto

    @property
    def start_offset(self) -> Number:
        return self._start_offset

    @property
    def edge_id_sequence(self) -> Tuple[str, ...]:
        return self._edge_id_sequence

    @property
    def end_offset(self) -> Number:
        return self._end_offset

    @property
    def edge_num(self) -> int:
        return len(self.edge_id_sequence)


class VehicleItinerary:
    def __init__(self, itinerary_base: VehicleItineraryBase, itinerary_index: int, edge_offset: Number,
                 current_edge_id: Optional[str] = None):
        """
        :param itinerary_base:
        :param itinerary_index:
        :param edge_offset:
        :param current_edge_id: Current edge should be an adjacent edge of itinerary_base[itinerary_index]
        """
        self._itinerary_base = itinerary_base
        self._itinerary_index = itinerary_index
        self._edge_offset = edge_offset
        self._current_edge_id = current_edge_id

    @property
    def itinerary_base(self) -> VehicleItineraryBase:
        return self._itinerary_base

    @property
    def start_offset(self) -> Number:
        return self.edge_offset

    @property
    def edge_id_sequence(self) -> Tuple[str, ...]:
        return self.itinerary_base.edge_id_sequence[self.itinerary_index:]

    @property
    def end_offset(self) -> Number:
        return self.itinerary_base.end_offset

    @property
    def edge_offset(self) -> Number:
        return self._edge_offset

    @property
    def itinerary_index(self) -> Number:
        return self._itinerary_index

    @property
    def current_edge_id(self) -> str:
        return self._current_edge_id if self._current_edge_id is not None else self.itinerary_base.edge_id_sequence[self.itinerary_index]


class VehicleDynamicState(ProtoClass):
    @singledispatchmethod
    def __init__(self, itinerary_index: int, edge_offset: Number, speed: Number, acceleration: Number,
                 waiting_time: Number, edge_id: str, position_2d: Vertex):
        self._itinerary_index = itinerary_index
        self._edge_offset = edge_offset
        self._speed = speed
        self._acceleration = acceleration
        self._waiting_time = waiting_time
        self._edge_id = edge_id
        self._position_2d = position_2d

    @__init__.register
    def __init__proto(self, proto: dynamic_state_pb2.VehicleDynamicState):
        self.__init__(proto.itinerary_index, proto.edge_offset, proto.speed, proto.acceleration,
                      proto.waiting_time, proto.edge_id, Vertex(proto.position_2d))

    def dump(self) -> dynamic_state_pb2.VehicleDynamicState:
        proto = dynamic_state_pb2.VehicleDynamicState()
        proto.itinerary_index = self.itinerary_index
        proto.edge_offset = self.edge_offset
        proto.speed = self.speed
        proto.acceleration = self.acceleration
        proto.waiting_time = self.waiting_time
        proto.edge_id = self.edge_id
        proto.position_2d.CopyFrom(self.position_2d.dump())
        return proto

    @property
    def itinerary_index(self) -> int:
        return self._itinerary_index

    @property
    def edge_offset(self) -> Number:
        return self._edge_offset

    @property
    def speed(self) -> Number:
        return self._speed

    @property
    def acceleration(self) -> Number:
        return self._acceleration

    @property
    def edge_id(self) -> str:
        return self._edge_id

    @property
    def position_2d(self) -> Vertex:
        return self._position_2d

    @property
    def waiting_time(self) -> Number:
        return self._waiting_time


class VehicleState:
    @singledispatchmethod
    def __init__(self, shape: VehicleShape, itinerary_base: VehicleItineraryBase, dynamic_state: VehicleDynamicState):
        self._shape = shape
        self._itinerary_base = itinerary_base
        self._dynamic_state = dynamic_state

    @property
    def shape(self) -> VehicleShape:
        return self._shape

    @property
    def itinerary_base(self) -> VehicleItineraryBase:
        return self._itinerary_base

    @property
    def dynamic_state(self) -> VehicleDynamicState:
        return self._dynamic_state

    @property
    def itinerary(self) -> VehicleItinerary:
        return VehicleItinerary(self.itinerary_base, self.dynamic_state.itinerary_index, self.dynamic_state.edge_offset,
                                self.dynamic_state.edge_id)

    @property
    def edge_id(self) -> str:
        return self.itinerary.current_edge_id

    @property
    def position(self) -> Position:
        return Position(self.edge_id, self.edge_offset)

    @property
    def position_2d(self) -> Vertex:
        return self.dynamic_state.position_2d

    @property
    def edge_offset(self) -> Number:
        return self.itinerary.edge_offset

    @property
    def speed(self) -> Number:
        return self.dynamic_state.speed

    @property
    def acceleration(self) -> Number:
        return self.dynamic_state.acceleration

    def __str__(self):
        return f'VehicleState({self.edge_id}, {self.edge_offset})'

    def __repr__(self):
        return str(self)
