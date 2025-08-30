# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros2web_interfaces:srv/HTTP.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'body'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HTTP_Request(type):
    """Metaclass of message 'HTTP_Request'."""

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
                'ros2web_interfaces.srv.HTTP_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__http__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__http__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__http__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__http__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__http__request

            from ros2web_interfaces.msg import BodyPart
            if BodyPart.__class__._TYPE_SUPPORT is None:
                BodyPart.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class HTTP_Request(metaclass=Metaclass_HTTP_Request):
    """Message class 'HTTP_Request'."""

    __slots__ = [
        '_headers',
        '_method',
        '_path',
        '_query',
        '_srv_name',
        '_route',
        '_body',
        '_text',
        '_content_type',
        '_charset',
        '_multipart',
    ]

    _fields_and_field_types = {
        'headers': 'sequence<string>',
        'method': 'string',
        'path': 'string',
        'query': 'string',
        'srv_name': 'string',
        'route': 'string',
        'body': 'sequence<uint8>',
        'text': 'string',
        'content_type': 'string',
        'charset': 'string',
        'multipart': 'sequence<ros2web_interfaces/BodyPart>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint8')),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['ros2web_interfaces', 'msg'], 'BodyPart')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.headers = kwargs.get('headers', [])
        self.method = kwargs.get('method', str())
        self.path = kwargs.get('path', str())
        self.query = kwargs.get('query', str())
        self.srv_name = kwargs.get('srv_name', str())
        self.route = kwargs.get('route', str())
        self.body = array.array('B', kwargs.get('body', []))
        self.text = kwargs.get('text', str())
        self.content_type = kwargs.get('content_type', str())
        self.charset = kwargs.get('charset', str())
        self.multipart = kwargs.get('multipart', [])

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
        if self.headers != other.headers:
            return False
        if self.method != other.method:
            return False
        if self.path != other.path:
            return False
        if self.query != other.query:
            return False
        if self.srv_name != other.srv_name:
            return False
        if self.route != other.route:
            return False
        if self.body != other.body:
            return False
        if self.text != other.text:
            return False
        if self.content_type != other.content_type:
            return False
        if self.charset != other.charset:
            return False
        if self.multipart != other.multipart:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def headers(self):
        """Message field 'headers'."""
        return self._headers

    @headers.setter
    def headers(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'headers' field must be a set or sequence and each value of type 'str'"
        self._headers = value

    @builtins.property
    def method(self):
        """Message field 'method'."""
        return self._method

    @method.setter
    def method(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'method' field must be of type 'str'"
        self._method = value

    @builtins.property
    def path(self):
        """Message field 'path'."""
        return self._path

    @path.setter
    def path(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'path' field must be of type 'str'"
        self._path = value

    @builtins.property
    def query(self):
        """Message field 'query'."""
        return self._query

    @query.setter
    def query(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'query' field must be of type 'str'"
        self._query = value

    @builtins.property
    def srv_name(self):
        """Message field 'srv_name'."""
        return self._srv_name

    @srv_name.setter
    def srv_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'srv_name' field must be of type 'str'"
        self._srv_name = value

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
    def body(self):
        """Message field 'body'."""
        return self._body

    @body.setter
    def body(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'B', \
                "The 'body' array.array() must have the type code of 'B'"
            self._body = value
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
                "The 'body' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 255]"
        self._body = array.array('B', value)

    @builtins.property
    def text(self):
        """Message field 'text'."""
        return self._text

    @text.setter
    def text(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'text' field must be of type 'str'"
        self._text = value

    @builtins.property
    def content_type(self):
        """Message field 'content_type'."""
        return self._content_type

    @content_type.setter
    def content_type(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'content_type' field must be of type 'str'"
        self._content_type = value

    @builtins.property
    def charset(self):
        """Message field 'charset'."""
        return self._charset

    @charset.setter
    def charset(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'charset' field must be of type 'str'"
        self._charset = value

    @builtins.property
    def multipart(self):
        """Message field 'multipart'."""
        return self._multipart

    @multipart.setter
    def multipart(self, value):
        if __debug__:
            from ros2web_interfaces.msg import BodyPart
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
                 all(isinstance(v, BodyPart) for v in value) and
                 True), \
                "The 'multipart' field must be a set or sequence and each value of type 'BodyPart'"
        self._multipart = value


# Import statements for member types

# Member 'body'
# already imported above
# import array

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_HTTP_Response(type):
    """Metaclass of message 'HTTP_Response'."""

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
                'ros2web_interfaces.srv.HTTP_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__http__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__http__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__http__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__http__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__http__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class HTTP_Response(metaclass=Metaclass_HTTP_Response):
    """Message class 'HTTP_Response'."""

    __slots__ = [
        '_status',
        '_reason',
        '_body',
        '_text',
        '_content_type',
        '_charset',
        '_file_path',
    ]

    _fields_and_field_types = {
        'status': 'uint16',
        'reason': 'string',
        'body': 'sequence<uint8>',
        'text': 'string',
        'content_type': 'string',
        'charset': 'string',
        'file_path': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint8')),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        self.reason = kwargs.get('reason', str())
        self.body = array.array('B', kwargs.get('body', []))
        self.text = kwargs.get('text', str())
        self.content_type = kwargs.get('content_type', str())
        self.charset = kwargs.get('charset', str())
        self.file_path = kwargs.get('file_path', str())

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
        if self.status != other.status:
            return False
        if self.reason != other.reason:
            return False
        if self.body != other.body:
            return False
        if self.text != other.text:
            return False
        if self.content_type != other.content_type:
            return False
        if self.charset != other.charset:
            return False
        if self.file_path != other.file_path:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'status' field must be an unsigned integer in [0, 65535]"
        self._status = value

    @builtins.property
    def reason(self):
        """Message field 'reason'."""
        return self._reason

    @reason.setter
    def reason(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'reason' field must be of type 'str'"
        self._reason = value

    @builtins.property
    def body(self):
        """Message field 'body'."""
        return self._body

    @body.setter
    def body(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'B', \
                "The 'body' array.array() must have the type code of 'B'"
            self._body = value
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
                "The 'body' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 255]"
        self._body = array.array('B', value)

    @builtins.property
    def text(self):
        """Message field 'text'."""
        return self._text

    @text.setter
    def text(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'text' field must be of type 'str'"
        self._text = value

    @builtins.property
    def content_type(self):
        """Message field 'content_type'."""
        return self._content_type

    @content_type.setter
    def content_type(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'content_type' field must be of type 'str'"
        self._content_type = value

    @builtins.property
    def charset(self):
        """Message field 'charset'."""
        return self._charset

    @charset.setter
    def charset(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'charset' field must be of type 'str'"
        self._charset = value

    @builtins.property
    def file_path(self):
        """Message field 'file_path'."""
        return self._file_path

    @file_path.setter
    def file_path(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'file_path' field must be of type 'str'"
        self._file_path = value


class Metaclass_HTTP(type):
    """Metaclass of service 'HTTP'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ros2web_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2web_interfaces.srv.HTTP')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__http

            from ros2web_interfaces.srv import _http
            if _http.Metaclass_HTTP_Request._TYPE_SUPPORT is None:
                _http.Metaclass_HTTP_Request.__import_type_support__()
            if _http.Metaclass_HTTP_Response._TYPE_SUPPORT is None:
                _http.Metaclass_HTTP_Response.__import_type_support__()


class HTTP(metaclass=Metaclass_HTTP):
    from ros2web_interfaces.srv._http import HTTP_Request as Request
    from ros2web_interfaces.srv._http import HTTP_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
