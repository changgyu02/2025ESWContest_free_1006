# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros2web_interfaces:msg/WSMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_WSMsg(type):
    """Metaclass of message 'WSMsg'."""

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
            module = import_type_support('ros2web_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2web_interfaces.msg.WSMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ws_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ws_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ws_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ws_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ws_msg

            from ros2web_interfaces.msg import WSMsgData
            if WSMsgData.__class__._TYPE_SUPPORT is None:
                WSMsgData.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class WSMsg(metaclass=Metaclass_WSMsg):
    """Message class 'WSMsg'."""

    __slots__ = [
        '_route',
        '_ws_id',
        '_type',
        '_data',
        '_extra',
    ]

    _fields_and_field_types = {
        'route': 'string',
        'ws_id': 'string',
        'type': 'uint8',
        'data': 'ros2web_interfaces/WSMsgData',
        'extra': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ros2web_interfaces', 'msg'], 'WSMsgData'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.route = kwargs.get('route', str())
        self.ws_id = kwargs.get('ws_id', str())
        self.type = kwargs.get('type', int())
        from ros2web_interfaces.msg import WSMsgData
        self.data = kwargs.get('data', WSMsgData())
        self.extra = kwargs.get('extra', str())

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
        if self.route != other.route:
            return False
        if self.ws_id != other.ws_id:
            return False
        if self.type != other.type:
            return False
        if self.data != other.data:
            return False
        if self.extra != other.extra:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def route(self):
        """Message field 'route'."""
        return self._route

    @route.setter
    def route(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'route' field must be of type 'str'"
        self._route = value

    @builtins.property
    def ws_id(self):
        """Message field 'ws_id'."""
        return self._ws_id

    @ws_id.setter
    def ws_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'ws_id' field must be of type 'str'"
        self._ws_id = value

    @builtins.property  # noqa: A003
    def type(self):  # noqa: A003
        """Message field 'type'."""
        return self._type

    @type.setter  # noqa: A003
    def type(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'type' field must be an unsigned integer in [0, 255]"
        self._type = value

    @builtins.property
    def data(self):
        """Message field 'data'."""
        return self._data

    @data.setter
    def data(self, value):
        if __debug__:
            from ros2web_interfaces.msg import WSMsgData
            assert \
                isinstance(value, WSMsgData), \
                "The 'data' field must be a sub message of type 'WSMsgData'"
        self._data = value

    @builtins.property
    def extra(self):
        """Message field 'extra'."""
        return self._extra

    @extra.setter
    def extra(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'extra' field must be of type 'str'"
        self._extra = value
