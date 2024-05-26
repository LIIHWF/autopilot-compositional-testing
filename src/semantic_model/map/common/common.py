from src.common.types import *
from src.semantic_model.map.common.proto import common_pb2


RJType = Enum('RJType', {name: id_ for name, id_ in common_pb2.RJType.items()})
TurnType = Enum('TurnType', {name: id_ for name, id_ in common_pb2.TurnType.items()})
StaticObjectType = Enum('StaticObjectType', {name: id_ for name, id_ in common_pb2.StaticObjectType.items()})

