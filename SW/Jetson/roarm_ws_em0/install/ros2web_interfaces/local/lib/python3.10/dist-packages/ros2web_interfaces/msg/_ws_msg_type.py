# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros2web_interfaces:msg/WSMsgType.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_WSMsgType(type):
    """Metaclass of message 'WSMsgType'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'CONTINUATION': 0,
        'TEXT': 1,
        'BINARY': 2,
        'PING': 11,
        'PONG': 12,
        'CLOSE': 21,
        'CLOSING': 22,
        'CLOSED': 23,
        'ERROR': 31,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ros2web_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2web_interfaces.msg.WSMsgType')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ws_msg_type
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ws_msg_type
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ws_msg_type
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ws_msg_type
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ws_msg_type

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'CONTINUATION': cls.__constants['CONTINUATION'],
            'TEXT': cls.__constants['TEXT'],
            'BINARY': cls.__constants['BINARY'],
            'PING': cls.__constants['PING'],
            'PONG': cls.__constants['PONG'],
            'CLOSE': cls.__constants['CLOSE'],
            'CLOSING': cls.__constants['CLOSING'],
            'CLOSED': cls.__constants['CLOSED'],
            'ERROR': cls.__constants['ERROR'],
        }

    @property
    def CONTINUATION(self):
        """Message constant 'CONTINUATION'."""
        return Metaclass_WSMsgType.__constants['CONTINUATION']

    @property
    def TEXT(self):
        """Message constant 'TEXT'."""
        return Metaclass_WSMsgType.__constants['TEXT']

    @property
    def BINARY(self):
        """Message constant 'BINARY'."""
        return Metaclass_WSMsgType.__constants['BINARY']

    @property
    def PING(self):
        """Message constant 'PING'."""
        return Metaclass_WSMsgType.__constants['PING']

    @property
    def PONG(self):
        """Message constant 'PONG'."""
        return Metaclass_WSMsgType.__constants['PONG']

    @property
    def CLOSE(self):
        """Message constant 'CLOSE'."""
        return Metaclass_WSMsgType.__constants['CLOSE']

    @property
    def CLOSING(self):
        """Message constant 'CLOSING'."""
        return Metaclass_WSMsgType.__constants['CLOSING']

    @property
    def CLOSED(self):
        """Message constant 'CLOSED'."""
        return Metaclass_WSMsgType.__constants['CLOSED']

    @property
    def ERROR(self):
        """Message constant 'ERROR'."""
        return Metaclass_WSMsgType.__constants['ERROR']


class WSMsgType(metaclass=Metaclass_WSMsgType):
    """
    Message class 'WSMsgType'.

    Constants:
      CONTINUATION
      TEXT
      BINARY
      PING
      PONG
      CLOSE
      CLOSING
      CLOSED
      ERROR
    """

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)
