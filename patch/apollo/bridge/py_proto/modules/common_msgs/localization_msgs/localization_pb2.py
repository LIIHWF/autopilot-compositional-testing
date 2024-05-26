# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/common_msgs/localization_msgs/localization.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common_msgs.basic_msgs import header_pb2 as modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2
from modules.common_msgs.basic_msgs import geometry_pb2 as modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2
from modules.common_msgs.basic_msgs import pnc_point_pb2 as modules_dot_common__msgs_dot_basic__msgs_dot_pnc__point__pb2
from modules.common_msgs.localization_msgs import localization_status_pb2 as modules_dot_common__msgs_dot_localization__msgs_dot_localization__status__pb2
from modules.common_msgs.localization_msgs import pose_pb2 as modules_dot_common__msgs_dot_localization__msgs_dot_pose__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/common_msgs/localization_msgs/localization.proto',
  package='apollo.localization',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n8modules/common_msgs/localization_msgs/localization.proto\x12\x13\x61pollo.localization\x1a+modules/common_msgs/basic_msgs/header.proto\x1a-modules/common_msgs/basic_msgs/geometry.proto\x1a.modules/common_msgs/basic_msgs/pnc_point.proto\x1a?modules/common_msgs/localization_msgs/localization_status.proto\x1a\x30modules/common_msgs/localization_msgs/pose.proto\"\xa4\x02\n\x0bUncertainty\x12\x30\n\x10position_std_dev\x18\x01 \x01(\x0b\x32\x16.apollo.common.Point3D\x12\x33\n\x13orientation_std_dev\x18\x02 \x01(\x0b\x32\x16.apollo.common.Point3D\x12\x37\n\x17linear_velocity_std_dev\x18\x03 \x01(\x0b\x32\x16.apollo.common.Point3D\x12;\n\x1blinear_acceleration_std_dev\x18\x04 \x01(\x0b\x32\x16.apollo.common.Point3D\x12\x38\n\x18\x61ngular_velocity_std_dev\x18\x05 \x01(\x0b\x32\x16.apollo.common.Point3D\"\xe5\x02\n\x14LocalizationEstimate\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12\'\n\x04pose\x18\x02 \x01(\x0b\x32\x19.apollo.localization.Pose\x12\x35\n\x0buncertainty\x18\x03 \x01(\x0b\x32 .apollo.localization.Uncertainty\x12\x18\n\x10measurement_time\x18\x04 \x01(\x01\x12\x38\n\x10trajectory_point\x18\x05 \x03(\x0b\x32\x1e.apollo.common.TrajectoryPoint\x12\x32\n\nmsf_status\x18\x06 \x01(\x0b\x32\x1e.apollo.localization.MsfStatus\x12>\n\rsensor_status\x18\x07 \x01(\x0b\x32\'.apollo.localization.MsfSensorMsgStatus\"\x9f\x02\n\x12LocalizationStatus\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12\x38\n\rfusion_status\x18\x02 \x01(\x0e\x32!.apollo.localization.MeasureState\x12:\n\x0bgnss_status\x18\x03 \x01(\x0e\x32!.apollo.localization.MeasureStateB\x02\x18\x01\x12;\n\x0clidar_status\x18\x04 \x01(\x0e\x32!.apollo.localization.MeasureStateB\x02\x18\x01\x12\x18\n\x10measurement_time\x18\x05 \x01(\x01\x12\x15\n\rstate_message\x18\x06 \x01(\t*T\n\x0cMeasureState\x12\x06\n\x02OK\x10\x00\x12\x0c\n\x08WARNNING\x10\x01\x12\t\n\x05\x45RROR\x10\x02\x12\x12\n\x0e\x43RITICAL_ERROR\x10\x03\x12\x0f\n\x0b\x46\x41TAL_ERROR\x10\x04'
  ,
  dependencies=[modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2.DESCRIPTOR,modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2.DESCRIPTOR,modules_dot_common__msgs_dot_basic__msgs_dot_pnc__point__pb2.DESCRIPTOR,modules_dot_common__msgs_dot_localization__msgs_dot_localization__status__pb2.DESCRIPTOR,modules_dot_common__msgs_dot_localization__msgs_dot_pose__pb2.DESCRIPTOR,])

_MEASURESTATE = _descriptor.EnumDescriptor(
  name='MeasureState',
  full_name='apollo.localization.MeasureState',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='OK', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='WARNNING', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ERROR', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CRITICAL_ERROR', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='FATAL_ERROR', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1281,
  serialized_end=1365,
)
_sym_db.RegisterEnumDescriptor(_MEASURESTATE)

MeasureState = enum_type_wrapper.EnumTypeWrapper(_MEASURESTATE)
OK = 0
WARNNING = 1
ERROR = 2
CRITICAL_ERROR = 3
FATAL_ERROR = 4



_UNCERTAINTY = _descriptor.Descriptor(
  name='Uncertainty',
  full_name='apollo.localization.Uncertainty',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='position_std_dev', full_name='apollo.localization.Uncertainty.position_std_dev', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='orientation_std_dev', full_name='apollo.localization.Uncertainty.orientation_std_dev', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='linear_velocity_std_dev', full_name='apollo.localization.Uncertainty.linear_velocity_std_dev', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='linear_acceleration_std_dev', full_name='apollo.localization.Uncertainty.linear_acceleration_std_dev', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='angular_velocity_std_dev', full_name='apollo.localization.Uncertainty.angular_velocity_std_dev', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=337,
  serialized_end=629,
)


_LOCALIZATIONESTIMATE = _descriptor.Descriptor(
  name='LocalizationEstimate',
  full_name='apollo.localization.LocalizationEstimate',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.localization.LocalizationEstimate.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='pose', full_name='apollo.localization.LocalizationEstimate.pose', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='uncertainty', full_name='apollo.localization.LocalizationEstimate.uncertainty', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='measurement_time', full_name='apollo.localization.LocalizationEstimate.measurement_time', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='trajectory_point', full_name='apollo.localization.LocalizationEstimate.trajectory_point', index=4,
      number=5, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='msf_status', full_name='apollo.localization.LocalizationEstimate.msf_status', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='sensor_status', full_name='apollo.localization.LocalizationEstimate.sensor_status', index=6,
      number=7, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=632,
  serialized_end=989,
)


_LOCALIZATIONSTATUS = _descriptor.Descriptor(
  name='LocalizationStatus',
  full_name='apollo.localization.LocalizationStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.localization.LocalizationStatus.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='fusion_status', full_name='apollo.localization.LocalizationStatus.fusion_status', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='gnss_status', full_name='apollo.localization.LocalizationStatus.gnss_status', index=2,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\030\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='lidar_status', full_name='apollo.localization.LocalizationStatus.lidar_status', index=3,
      number=4, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\030\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='measurement_time', full_name='apollo.localization.LocalizationStatus.measurement_time', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='state_message', full_name='apollo.localization.LocalizationStatus.state_message', index=5,
      number=6, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=992,
  serialized_end=1279,
)

_UNCERTAINTY.fields_by_name['position_std_dev'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2._POINT3D
_UNCERTAINTY.fields_by_name['orientation_std_dev'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2._POINT3D
_UNCERTAINTY.fields_by_name['linear_velocity_std_dev'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2._POINT3D
_UNCERTAINTY.fields_by_name['linear_acceleration_std_dev'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2._POINT3D
_UNCERTAINTY.fields_by_name['angular_velocity_std_dev'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2._POINT3D
_LOCALIZATIONESTIMATE.fields_by_name['header'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2._HEADER
_LOCALIZATIONESTIMATE.fields_by_name['pose'].message_type = modules_dot_common__msgs_dot_localization__msgs_dot_pose__pb2._POSE
_LOCALIZATIONESTIMATE.fields_by_name['uncertainty'].message_type = _UNCERTAINTY
_LOCALIZATIONESTIMATE.fields_by_name['trajectory_point'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_pnc__point__pb2._TRAJECTORYPOINT
_LOCALIZATIONESTIMATE.fields_by_name['msf_status'].message_type = modules_dot_common__msgs_dot_localization__msgs_dot_localization__status__pb2._MSFSTATUS
_LOCALIZATIONESTIMATE.fields_by_name['sensor_status'].message_type = modules_dot_common__msgs_dot_localization__msgs_dot_localization__status__pb2._MSFSENSORMSGSTATUS
_LOCALIZATIONSTATUS.fields_by_name['header'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2._HEADER
_LOCALIZATIONSTATUS.fields_by_name['fusion_status'].enum_type = _MEASURESTATE
_LOCALIZATIONSTATUS.fields_by_name['gnss_status'].enum_type = _MEASURESTATE
_LOCALIZATIONSTATUS.fields_by_name['lidar_status'].enum_type = _MEASURESTATE
DESCRIPTOR.message_types_by_name['Uncertainty'] = _UNCERTAINTY
DESCRIPTOR.message_types_by_name['LocalizationEstimate'] = _LOCALIZATIONESTIMATE
DESCRIPTOR.message_types_by_name['LocalizationStatus'] = _LOCALIZATIONSTATUS
DESCRIPTOR.enum_types_by_name['MeasureState'] = _MEASURESTATE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Uncertainty = _reflection.GeneratedProtocolMessageType('Uncertainty', (_message.Message,), {
  'DESCRIPTOR' : _UNCERTAINTY,
  '__module__' : 'modules.common_msgs.localization_msgs.localization_pb2'
  # @@protoc_insertion_point(class_scope:apollo.localization.Uncertainty)
  })
_sym_db.RegisterMessage(Uncertainty)

LocalizationEstimate = _reflection.GeneratedProtocolMessageType('LocalizationEstimate', (_message.Message,), {
  'DESCRIPTOR' : _LOCALIZATIONESTIMATE,
  '__module__' : 'modules.common_msgs.localization_msgs.localization_pb2'
  # @@protoc_insertion_point(class_scope:apollo.localization.LocalizationEstimate)
  })
_sym_db.RegisterMessage(LocalizationEstimate)

LocalizationStatus = _reflection.GeneratedProtocolMessageType('LocalizationStatus', (_message.Message,), {
  'DESCRIPTOR' : _LOCALIZATIONSTATUS,
  '__module__' : 'modules.common_msgs.localization_msgs.localization_pb2'
  # @@protoc_insertion_point(class_scope:apollo.localization.LocalizationStatus)
  })
_sym_db.RegisterMessage(LocalizationStatus)


_LOCALIZATIONSTATUS.fields_by_name['gnss_status']._options = None
_LOCALIZATIONSTATUS.fields_by_name['lidar_status']._options = None
# @@protoc_insertion_point(module_scope)
