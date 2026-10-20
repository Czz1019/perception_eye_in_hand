# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ur7e_perception_planning:msg/PlanningMap.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'nvblox_distance'
# Member 'planning_distance'
# Member 'free_source'
# Member 'compressed_data'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'dimensions'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PlanningMap(type):
    """Metaclass of message 'PlanningMap'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ur7e_perception_planning')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ur7e_perception_planning.msg.PlanningMap')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__planning_map
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__planning_map
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__planning_map
            cls._TYPE_SUPPORT = module.type_support_msg__msg__planning_map
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__planning_map

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PlanningMap(metaclass=Metaclass_PlanningMap):
    """Message class 'PlanningMap'."""

    __slots__ = [
        '_header',
        '_epoch',
        '_sequence',
        '_origin',
        '_resolution',
        '_dimensions',
        '_unknown_distance',
        '_nvblox_distance',
        '_planning_distance',
        '_free_source',
        '_compressed_data',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'epoch': 'uint64',
        'sequence': 'uint64',
        'origin': 'geometry_msgs/Point',
        'resolution': 'float',
        'dimensions': 'uint32[3]',
        'unknown_distance': 'float',
        'nvblox_distance': 'sequence<float>',
        'planning_distance': 'sequence<float>',
        'free_source': 'sequence<uint8>',
        'compressed_data': 'sequence<uint8>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint32'), 3),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint8')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint8')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.epoch = kwargs.get('epoch', int())
        self.sequence = kwargs.get('sequence', int())
        from geometry_msgs.msg import Point
        self.origin = kwargs.get('origin', Point())
        self.resolution = kwargs.get('resolution', float())
        if 'dimensions' not in kwargs:
            self.dimensions = numpy.zeros(3, dtype=numpy.uint32)
        else:
            self.dimensions = kwargs.get('dimensions')
        self.unknown_distance = kwargs.get('unknown_distance', float())
        self.nvblox_distance = array.array('f', kwargs.get('nvblox_distance', []))
        self.planning_distance = array.array('f', kwargs.get('planning_distance', []))
        self.free_source = array.array('B', kwargs.get('free_source', []))
        self.compressed_data = array.array('B', kwargs.get('compressed_data', []))

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.epoch != other.epoch:
            return False
        if self.sequence != other.sequence:
            return False
        if self.origin != other.origin:
            return False
        if self.resolution != other.resolution:
            return False
        if any(self.dimensions != other.dimensions):
            return False
        if self.unknown_distance != other.unknown_distance:
            return False
        if self.nvblox_distance != other.nvblox_distance:
            return False
        if self.planning_distance != other.planning_distance:
            return False
        if self.free_source != other.free_source:
            return False
        if self.compressed_data != other.compressed_data:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def epoch(self):
        """Message field 'epoch'."""
        return self._epoch

    @epoch.setter
    def epoch(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'epoch' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'epoch' field must be an unsigned integer in [0, 18446744073709551615]"
        self._epoch = value

    @builtins.property
    def sequence(self):
        """Message field 'sequence'."""
        return self._sequence

    @sequence.setter
    def sequence(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sequence' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'sequence' field must be an unsigned integer in [0, 18446744073709551615]"
        self._sequence = value

    @builtins.property
    def origin(self):
        """Message field 'origin'."""
        return self._origin

    @origin.setter
    def origin(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'origin' field must be a sub message of type 'Point'"
        self._origin = value

    @builtins.property
    def resolution(self):
        """Message field 'resolution'."""
        return self._resolution

    @resolution.setter
    def resolution(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'resolution' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'resolution' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._resolution = value

    @builtins.property
    def dimensions(self):
        """Message field 'dimensions'."""
        return self._dimensions

    @dimensions.setter
    def dimensions(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint32, \
                "The 'dimensions' numpy.ndarray() must have the dtype of 'numpy.uint32'"
            assert value.size == 3, \
                "The 'dimensions' numpy.ndarray() must have a size of 3"
            self._dimensions = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 4294967296 for val in value)), \
                "The 'dimensions' field must be a set or sequence with length 3 and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._dimensions = numpy.array(value, dtype=numpy.uint32)

    @builtins.property
    def unknown_distance(self):
        """Message field 'unknown_distance'."""
        return self._unknown_distance

    @unknown_distance.setter
    def unknown_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'unknown_distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'unknown_distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._unknown_distance = value

    @builtins.property
    def nvblox_distance(self):
        """Message field 'nvblox_distance'."""
        return self._nvblox_distance

    @nvblox_distance.setter
    def nvblox_distance(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'nvblox_distance' array.array() must have the type code of 'f'"
            self._nvblox_distance = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'nvblox_distance' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._nvblox_distance = array.array('f', value)

    @builtins.property
    def planning_distance(self):
        """Message field 'planning_distance'."""
        return self._planning_distance

    @planning_distance.setter
    def planning_distance(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'planning_distance' array.array() must have the type code of 'f'"
            self._planning_distance = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'planning_distance' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._planning_distance = array.array('f', value)

    @builtins.property
    def free_source(self):
        """Message field 'free_source'."""
        return self._free_source

    @free_source.setter
    def free_source(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'B', \
                "The 'free_source' array.array() must have the type code of 'B'"
            self._free_source = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'free_source' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 255]"
        self._free_source = array.array('B', value)

    @builtins.property
    def compressed_data(self):
        """Message field 'compressed_data'."""
        return self._compressed_data

    @compressed_data.setter
    def compressed_data(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'B', \
                "The 'compressed_data' array.array() must have the type code of 'B'"
            self._compressed_data = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'compressed_data' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 255]"
        self._compressed_data = array.array('B', value)
