# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/common_msgs/dreamview_msgs/chart.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common_msgs.basic_msgs import geometry_pb2 as modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/common_msgs/dreamview_msgs/chart.proto',
  package='apollo.dreamview',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n.modules/common_msgs/dreamview_msgs/chart.proto\x12\x10\x61pollo.dreamview\x1a-modules/common_msgs/basic_msgs/geometry.proto\"\xaa\x02\n\x07Options\x12\x1c\n\x0elegend_display\x18\x01 \x01(\x08:\x04true\x12)\n\x01x\x18\x02 \x01(\x0b\x32\x1e.apollo.dreamview.Options.Axis\x12)\n\x01y\x18\x03 \x01(\x0b\x32\x1e.apollo.dreamview.Options.Axis\x12\x14\n\x0c\x61spect_ratio\x18\x04 \x01(\x01\x12\"\n\x13sync_xy_window_size\x18\x05 \x01(\x08:\x05\x66\x61lse\x1aq\n\x04\x41xis\x12\x0b\n\x03min\x18\x01 \x01(\x01\x12\x0b\n\x03max\x18\x02 \x01(\x01\x12\x14\n\x0clabel_string\x18\x03 \x01(\t\x12\x13\n\x0bwindow_size\x18\x04 \x01(\x01\x12\x11\n\tstep_size\x18\x05 \x01(\x01\x12\x11\n\tmid_value\x18\x06 \x01(\x01\"\xd0\x01\n\x04Line\x12\r\n\x05label\x18\x01 \x01(\t\x12#\n\x14hide_label_in_legend\x18\x02 \x01(\x08:\x05\x66\x61lse\x12%\n\x05point\x18\x03 \x03(\x0b\x32\x16.apollo.common.Point2D\x12:\n\nproperties\x18\x04 \x03(\x0b\x32&.apollo.dreamview.Line.PropertiesEntry\x1a\x31\n\x0fPropertiesEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\r\n\x05value\x18\x02 \x01(\t:\x02\x38\x01\"\xd6\x01\n\x07Polygon\x12\r\n\x05label\x18\x01 \x01(\t\x12#\n\x14hide_label_in_legend\x18\x02 \x01(\x08:\x05\x66\x61lse\x12%\n\x05point\x18\x03 \x03(\x0b\x32\x16.apollo.common.Point2D\x12=\n\nproperties\x18\x04 \x03(\x0b\x32).apollo.dreamview.Polygon.PropertiesEntry\x1a\x31\n\x0fPropertiesEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\r\n\x05value\x18\x02 \x01(\t:\x02\x38\x01\"o\n\x03\x43\x61r\x12\r\n\x05label\x18\x01 \x01(\t\x12#\n\x14hide_label_in_legend\x18\x02 \x01(\x08:\x05\x66\x61lse\x12\t\n\x01x\x18\x03 \x01(\x01\x12\t\n\x01y\x18\x04 \x01(\x01\x12\x0f\n\x07heading\x18\x05 \x01(\x01\x12\r\n\x05\x63olor\x18\x06 \x01(\t\"\xb8\x01\n\x05\x43hart\x12\r\n\x05title\x18\x01 \x01(\t\x12*\n\x07options\x18\x02 \x01(\x0b\x32\x19.apollo.dreamview.Options\x12$\n\x04line\x18\x03 \x03(\x0b\x32\x16.apollo.dreamview.Line\x12*\n\x07polygon\x18\x04 \x03(\x0b\x32\x19.apollo.dreamview.Polygon\x12\"\n\x03\x63\x61r\x18\x05 \x03(\x0b\x32\x15.apollo.dreamview.Car'
  ,
  dependencies=[modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2.DESCRIPTOR,])




_OPTIONS_AXIS = _descriptor.Descriptor(
  name='Axis',
  full_name='apollo.dreamview.Options.Axis',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='min', full_name='apollo.dreamview.Options.Axis.min', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max', full_name='apollo.dreamview.Options.Axis.max', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='label_string', full_name='apollo.dreamview.Options.Axis.label_string', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='window_size', full_name='apollo.dreamview.Options.Axis.window_size', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='step_size', full_name='apollo.dreamview.Options.Axis.step_size', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='mid_value', full_name='apollo.dreamview.Options.Axis.mid_value', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
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
  serialized_start=301,
  serialized_end=414,
)

_OPTIONS = _descriptor.Descriptor(
  name='Options',
  full_name='apollo.dreamview.Options',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='legend_display', full_name='apollo.dreamview.Options.legend_display', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=True,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='x', full_name='apollo.dreamview.Options.x', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='y', full_name='apollo.dreamview.Options.y', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='aspect_ratio', full_name='apollo.dreamview.Options.aspect_ratio', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='sync_xy_window_size', full_name='apollo.dreamview.Options.sync_xy_window_size', index=4,
      number=5, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_OPTIONS_AXIS, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=116,
  serialized_end=414,
)


_LINE_PROPERTIESENTRY = _descriptor.Descriptor(
  name='PropertiesEntry',
  full_name='apollo.dreamview.Line.PropertiesEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.dreamview.Line.PropertiesEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.dreamview.Line.PropertiesEntry.value', index=1,
      number=2, type=9, cpp_type=9, label=1,
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
  serialized_options=b'8\001',
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=576,
  serialized_end=625,
)

_LINE = _descriptor.Descriptor(
  name='Line',
  full_name='apollo.dreamview.Line',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='label', full_name='apollo.dreamview.Line.label', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='hide_label_in_legend', full_name='apollo.dreamview.Line.hide_label_in_legend', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='point', full_name='apollo.dreamview.Line.point', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='properties', full_name='apollo.dreamview.Line.properties', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_LINE_PROPERTIESENTRY, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=417,
  serialized_end=625,
)


_POLYGON_PROPERTIESENTRY = _descriptor.Descriptor(
  name='PropertiesEntry',
  full_name='apollo.dreamview.Polygon.PropertiesEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.dreamview.Polygon.PropertiesEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.dreamview.Polygon.PropertiesEntry.value', index=1,
      number=2, type=9, cpp_type=9, label=1,
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
  serialized_options=b'8\001',
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=576,
  serialized_end=625,
)

_POLYGON = _descriptor.Descriptor(
  name='Polygon',
  full_name='apollo.dreamview.Polygon',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='label', full_name='apollo.dreamview.Polygon.label', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='hide_label_in_legend', full_name='apollo.dreamview.Polygon.hide_label_in_legend', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='point', full_name='apollo.dreamview.Polygon.point', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='properties', full_name='apollo.dreamview.Polygon.properties', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_POLYGON_PROPERTIESENTRY, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=628,
  serialized_end=842,
)


_CAR = _descriptor.Descriptor(
  name='Car',
  full_name='apollo.dreamview.Car',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='label', full_name='apollo.dreamview.Car.label', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='hide_label_in_legend', full_name='apollo.dreamview.Car.hide_label_in_legend', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='x', full_name='apollo.dreamview.Car.x', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='y', full_name='apollo.dreamview.Car.y', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='heading', full_name='apollo.dreamview.Car.heading', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='color', full_name='apollo.dreamview.Car.color', index=5,
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
  serialized_start=844,
  serialized_end=955,
)


_CHART = _descriptor.Descriptor(
  name='Chart',
  full_name='apollo.dreamview.Chart',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='title', full_name='apollo.dreamview.Chart.title', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='options', full_name='apollo.dreamview.Chart.options', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='line', full_name='apollo.dreamview.Chart.line', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='polygon', full_name='apollo.dreamview.Chart.polygon', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='car', full_name='apollo.dreamview.Chart.car', index=4,
      number=5, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=958,
  serialized_end=1142,
)

_OPTIONS_AXIS.containing_type = _OPTIONS
_OPTIONS.fields_by_name['x'].message_type = _OPTIONS_AXIS
_OPTIONS.fields_by_name['y'].message_type = _OPTIONS_AXIS
_LINE_PROPERTIESENTRY.containing_type = _LINE
_LINE.fields_by_name['point'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2._POINT2D
_LINE.fields_by_name['properties'].message_type = _LINE_PROPERTIESENTRY
_POLYGON_PROPERTIESENTRY.containing_type = _POLYGON
_POLYGON.fields_by_name['point'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2._POINT2D
_POLYGON.fields_by_name['properties'].message_type = _POLYGON_PROPERTIESENTRY
_CHART.fields_by_name['options'].message_type = _OPTIONS
_CHART.fields_by_name['line'].message_type = _LINE
_CHART.fields_by_name['polygon'].message_type = _POLYGON
_CHART.fields_by_name['car'].message_type = _CAR
DESCRIPTOR.message_types_by_name['Options'] = _OPTIONS
DESCRIPTOR.message_types_by_name['Line'] = _LINE
DESCRIPTOR.message_types_by_name['Polygon'] = _POLYGON
DESCRIPTOR.message_types_by_name['Car'] = _CAR
DESCRIPTOR.message_types_by_name['Chart'] = _CHART
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Options = _reflection.GeneratedProtocolMessageType('Options', (_message.Message,), {

  'Axis' : _reflection.GeneratedProtocolMessageType('Axis', (_message.Message,), {
    'DESCRIPTOR' : _OPTIONS_AXIS,
    '__module__' : 'modules.common_msgs.dreamview_msgs.chart_pb2'
    # @@protoc_insertion_point(class_scope:apollo.dreamview.Options.Axis)
    })
  ,
  'DESCRIPTOR' : _OPTIONS,
  '__module__' : 'modules.common_msgs.dreamview_msgs.chart_pb2'
  # @@protoc_insertion_point(class_scope:apollo.dreamview.Options)
  })
_sym_db.RegisterMessage(Options)
_sym_db.RegisterMessage(Options.Axis)

Line = _reflection.GeneratedProtocolMessageType('Line', (_message.Message,), {

  'PropertiesEntry' : _reflection.GeneratedProtocolMessageType('PropertiesEntry', (_message.Message,), {
    'DESCRIPTOR' : _LINE_PROPERTIESENTRY,
    '__module__' : 'modules.common_msgs.dreamview_msgs.chart_pb2'
    # @@protoc_insertion_point(class_scope:apollo.dreamview.Line.PropertiesEntry)
    })
  ,
  'DESCRIPTOR' : _LINE,
  '__module__' : 'modules.common_msgs.dreamview_msgs.chart_pb2'
  # @@protoc_insertion_point(class_scope:apollo.dreamview.Line)
  })
_sym_db.RegisterMessage(Line)
_sym_db.RegisterMessage(Line.PropertiesEntry)

Polygon = _reflection.GeneratedProtocolMessageType('Polygon', (_message.Message,), {

  'PropertiesEntry' : _reflection.GeneratedProtocolMessageType('PropertiesEntry', (_message.Message,), {
    'DESCRIPTOR' : _POLYGON_PROPERTIESENTRY,
    '__module__' : 'modules.common_msgs.dreamview_msgs.chart_pb2'
    # @@protoc_insertion_point(class_scope:apollo.dreamview.Polygon.PropertiesEntry)
    })
  ,
  'DESCRIPTOR' : _POLYGON,
  '__module__' : 'modules.common_msgs.dreamview_msgs.chart_pb2'
  # @@protoc_insertion_point(class_scope:apollo.dreamview.Polygon)
  })
_sym_db.RegisterMessage(Polygon)
_sym_db.RegisterMessage(Polygon.PropertiesEntry)

Car = _reflection.GeneratedProtocolMessageType('Car', (_message.Message,), {
  'DESCRIPTOR' : _CAR,
  '__module__' : 'modules.common_msgs.dreamview_msgs.chart_pb2'
  # @@protoc_insertion_point(class_scope:apollo.dreamview.Car)
  })
_sym_db.RegisterMessage(Car)

Chart = _reflection.GeneratedProtocolMessageType('Chart', (_message.Message,), {
  'DESCRIPTOR' : _CHART,
  '__module__' : 'modules.common_msgs.dreamview_msgs.chart_pb2'
  # @@protoc_insertion_point(class_scope:apollo.dreamview.Chart)
  })
_sym_db.RegisterMessage(Chart)


_LINE_PROPERTIESENTRY._options = None
_POLYGON_PROPERTIESENTRY._options = None
# @@protoc_insertion_point(module_scope)
