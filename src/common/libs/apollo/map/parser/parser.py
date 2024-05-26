from src.common.types import *
from src.common.libs.apollo.map.proto import map_pb2
from src.common.libs.apollo.map.proto import map_id_pb2
from google.protobuf import text_format


MapProto = map_pb2.Map
MapProtoId = map_id_pb2.Id


class ApolloMapParser:
    def __init__(self, proto_bytes: Union[bytes, str]):
        if isinstance(proto_bytes, str):
            self.proto: map_pb2.Map = text_format.Parse(proto_bytes, map_pb2.Map())
        else:    
            self.proto: map_pb2.Map = map_pb2.Map()
            self.proto.ParseFromString(proto_bytes)

    def dump(self) -> bytes:
        return self.proto.SerializeToString()

    def dumps(self) -> str:
        return str(self.proto)

    def copy(self) -> 'ApolloMapParser':
        map_proto = ApolloMapParser(self.dump())
        return map_proto
