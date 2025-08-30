# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros2web_interfaces:msg/HTTPStatusCode.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HTTPStatusCode(type):
    """Metaclass of message 'HTTPStatusCode'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'HTTP_OK': 200,
        'HTTP_ACCEPTED': 202,
        'HTTP_NON_AUTHORITATIVE_INFORMATION': 203,
        'HTTP_NO_CONTENT': 204,
        'HTTP_RESET_CONTENT': 205,
        'HTTP_PARTIAL_CONTENT': 206,
        'HTTP_MULTIPLE_CHOICES': 300,
        'HTTP_MOVED_PERMANENTLY': 301,
        'HTTP_FOUND': 302,
        'HTTP_SEE_OTHER': 303,
        'HTTP_NOT_MODIFIED': 304,
        'HTTP_USE_PROXY': 305,
        'HTTP_TEMPORARY_REDIRECT': 307,
        'HTTP_PERMANENT_REDIRECT': 308,
        'HTTP_BAD_REQUEST': 400,
        'HTTP_UNAUTHORIZED': 401,
        'HTTP_PAYMENT_REQUIRED': 402,
        'HTTP_FORBIDDEN': 403,
        'HTTP_NOT_FOUND': 404,
        'HTTP_METHOD_NOT_ALLOWED': 405,
        'HTTP_NOT_ACCEPTABLE': 406,
        'HTTP_PROXY_AUTHENTICATION_REQUIRED': 407,
        'HTTP_REQUEST_TIMEOUT': 408,
        'HTTP_CONFLICT': 409,
        'HTTP_GONE': 410,
        'HTTP_LENGTH_REQUIRED': 411,
        'HTTP_PRECONDITION_FAILED': 412,
        'HTTP_REQUEST_ENTITY_TOO_LARGE': 413,
        'HTTP_REQUEST_URI_TOOLONG': 414,
        'HTTP_UNSUPPORTED_MEDIA_TYPE': 415,
        'HTTP_REQUEST_RANGE_NOT_SATISFIABLE': 416,
        'HTTP_EXPECTATION_FAILED': 417,
        'HTTP_MISDIRECTED_REQUEST': 421,
        'HTTP_UNPROCESSABLE_ENTITY': 422,
        'HTTP_FAILED_DEPENDENCY': 424,
        'HTTP_UPGRADE_REQUIRED': 426,
        'HTTP_PRECONDITION_REQUIRED': 428,
        'HTTP_TOO_MANY_REQUESTS': 429,
        'HTTP_REQUEST_HEADER_FIELDS_TOO_LARGE': 431,
        'HTTP_UNAVAILABLE_FOR_LEGAL_REASONS': 451,
        'HTTP_INTERNAL_SERVER_ERROR': 500,
        'HTTP_NOT_IMPLEMENTED': 501,
        'HTTP_BAD_GATEWAY': 502,
        'HTTP_SERVICE_UNAVAILABLE': 503,
        'HTTP_GATEWAY_TIMEOUT': 504,
        'HTTP_VERSION_NOT_SUPPORTED': 505,
        'HTTP_VARIANT_ALSO_NEGOTIATES': 506,
        'HTTP_INSUFFICIENT_STORAGE': 507,
        'HTTP_NOT_EXTENDED': 510,
        'HTTP_NETWORK_AUTHENTICATION_REQUIRED': 511,
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
                'ros2web_interfaces.msg.HTTPStatusCode')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__http_status_code
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__http_status_code
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__http_status_code
            cls._TYPE_SUPPORT = module.type_support_msg__msg__http_status_code
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__http_status_code

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'HTTP_OK': cls.__constants['HTTP_OK'],
            'HTTP_ACCEPTED': cls.__constants['HTTP_ACCEPTED'],
            'HTTP_NON_AUTHORITATIVE_INFORMATION': cls.__constants['HTTP_NON_AUTHORITATIVE_INFORMATION'],
            'HTTP_NO_CONTENT': cls.__constants['HTTP_NO_CONTENT'],
            'HTTP_RESET_CONTENT': cls.__constants['HTTP_RESET_CONTENT'],
            'HTTP_PARTIAL_CONTENT': cls.__constants['HTTP_PARTIAL_CONTENT'],
            'HTTP_MULTIPLE_CHOICES': cls.__constants['HTTP_MULTIPLE_CHOICES'],
            'HTTP_MOVED_PERMANENTLY': cls.__constants['HTTP_MOVED_PERMANENTLY'],
            'HTTP_FOUND': cls.__constants['HTTP_FOUND'],
            'HTTP_SEE_OTHER': cls.__constants['HTTP_SEE_OTHER'],
            'HTTP_NOT_MODIFIED': cls.__constants['HTTP_NOT_MODIFIED'],
            'HTTP_USE_PROXY': cls.__constants['HTTP_USE_PROXY'],
            'HTTP_TEMPORARY_REDIRECT': cls.__constants['HTTP_TEMPORARY_REDIRECT'],
            'HTTP_PERMANENT_REDIRECT': cls.__constants['HTTP_PERMANENT_REDIRECT'],
            'HTTP_BAD_REQUEST': cls.__constants['HTTP_BAD_REQUEST'],
            'HTTP_UNAUTHORIZED': cls.__constants['HTTP_UNAUTHORIZED'],
            'HTTP_PAYMENT_REQUIRED': cls.__constants['HTTP_PAYMENT_REQUIRED'],
            'HTTP_FORBIDDEN': cls.__constants['HTTP_FORBIDDEN'],
            'HTTP_NOT_FOUND': cls.__constants['HTTP_NOT_FOUND'],
            'HTTP_METHOD_NOT_ALLOWED': cls.__constants['HTTP_METHOD_NOT_ALLOWED'],
            'HTTP_NOT_ACCEPTABLE': cls.__constants['HTTP_NOT_ACCEPTABLE'],
            'HTTP_PROXY_AUTHENTICATION_REQUIRED': cls.__constants['HTTP_PROXY_AUTHENTICATION_REQUIRED'],
            'HTTP_REQUEST_TIMEOUT': cls.__constants['HTTP_REQUEST_TIMEOUT'],
            'HTTP_CONFLICT': cls.__constants['HTTP_CONFLICT'],
            'HTTP_GONE': cls.__constants['HTTP_GONE'],
            'HTTP_LENGTH_REQUIRED': cls.__constants['HTTP_LENGTH_REQUIRED'],
            'HTTP_PRECONDITION_FAILED': cls.__constants['HTTP_PRECONDITION_FAILED'],
            'HTTP_REQUEST_ENTITY_TOO_LARGE': cls.__constants['HTTP_REQUEST_ENTITY_TOO_LARGE'],
            'HTTP_REQUEST_URI_TOOLONG': cls.__constants['HTTP_REQUEST_URI_TOOLONG'],
            'HTTP_UNSUPPORTED_MEDIA_TYPE': cls.__constants['HTTP_UNSUPPORTED_MEDIA_TYPE'],
            'HTTP_REQUEST_RANGE_NOT_SATISFIABLE': cls.__constants['HTTP_REQUEST_RANGE_NOT_SATISFIABLE'],
            'HTTP_EXPECTATION_FAILED': cls.__constants['HTTP_EXPECTATION_FAILED'],
            'HTTP_MISDIRECTED_REQUEST': cls.__constants['HTTP_MISDIRECTED_REQUEST'],
            'HTTP_UNPROCESSABLE_ENTITY': cls.__constants['HTTP_UNPROCESSABLE_ENTITY'],
            'HTTP_FAILED_DEPENDENCY': cls.__constants['HTTP_FAILED_DEPENDENCY'],
            'HTTP_UPGRADE_REQUIRED': cls.__constants['HTTP_UPGRADE_REQUIRED'],
            'HTTP_PRECONDITION_REQUIRED': cls.__constants['HTTP_PRECONDITION_REQUIRED'],
            'HTTP_TOO_MANY_REQUESTS': cls.__constants['HTTP_TOO_MANY_REQUESTS'],
            'HTTP_REQUEST_HEADER_FIELDS_TOO_LARGE': cls.__constants['HTTP_REQUEST_HEADER_FIELDS_TOO_LARGE'],
            'HTTP_UNAVAILABLE_FOR_LEGAL_REASONS': cls.__constants['HTTP_UNAVAILABLE_FOR_LEGAL_REASONS'],
            'HTTP_INTERNAL_SERVER_ERROR': cls.__constants['HTTP_INTERNAL_SERVER_ERROR'],
            'HTTP_NOT_IMPLEMENTED': cls.__constants['HTTP_NOT_IMPLEMENTED'],
            'HTTP_BAD_GATEWAY': cls.__constants['HTTP_BAD_GATEWAY'],
            'HTTP_SERVICE_UNAVAILABLE': cls.__constants['HTTP_SERVICE_UNAVAILABLE'],
            'HTTP_GATEWAY_TIMEOUT': cls.__constants['HTTP_GATEWAY_TIMEOUT'],
            'HTTP_VERSION_NOT_SUPPORTED': cls.__constants['HTTP_VERSION_NOT_SUPPORTED'],
            'HTTP_VARIANT_ALSO_NEGOTIATES': cls.__constants['HTTP_VARIANT_ALSO_NEGOTIATES'],
            'HTTP_INSUFFICIENT_STORAGE': cls.__constants['HTTP_INSUFFICIENT_STORAGE'],
            'HTTP_NOT_EXTENDED': cls.__constants['HTTP_NOT_EXTENDED'],
            'HTTP_NETWORK_AUTHENTICATION_REQUIRED': cls.__constants['HTTP_NETWORK_AUTHENTICATION_REQUIRED'],
        }

    @property
    def HTTP_OK(self):
        """Message constant 'HTTP_OK'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_OK']

    @property
    def HTTP_ACCEPTED(self):
        """Message constant 'HTTP_ACCEPTED'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_ACCEPTED']

    @property
    def HTTP_NON_AUTHORITATIVE_INFORMATION(self):
        """Message constant 'HTTP_NON_AUTHORITATIVE_INFORMATION'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_NON_AUTHORITATIVE_INFORMATION']

    @property
    def HTTP_NO_CONTENT(self):
        """Message constant 'HTTP_NO_CONTENT'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_NO_CONTENT']

    @property
    def HTTP_RESET_CONTENT(self):
        """Message constant 'HTTP_RESET_CONTENT'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_RESET_CONTENT']

    @property
    def HTTP_PARTIAL_CONTENT(self):
        """Message constant 'HTTP_PARTIAL_CONTENT'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_PARTIAL_CONTENT']

    @property
    def HTTP_MULTIPLE_CHOICES(self):
        """Message constant 'HTTP_MULTIPLE_CHOICES'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_MULTIPLE_CHOICES']

    @property
    def HTTP_MOVED_PERMANENTLY(self):
        """Message constant 'HTTP_MOVED_PERMANENTLY'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_MOVED_PERMANENTLY']

    @property
    def HTTP_FOUND(self):
        """Message constant 'HTTP_FOUND'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_FOUND']

    @property
    def HTTP_SEE_OTHER(self):
        """Message constant 'HTTP_SEE_OTHER'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_SEE_OTHER']

    @property
    def HTTP_NOT_MODIFIED(self):
        """Message constant 'HTTP_NOT_MODIFIED'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_NOT_MODIFIED']

    @property
    def HTTP_USE_PROXY(self):
        """Message constant 'HTTP_USE_PROXY'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_USE_PROXY']

    @property
    def HTTP_TEMPORARY_REDIRECT(self):
        """Message constant 'HTTP_TEMPORARY_REDIRECT'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_TEMPORARY_REDIRECT']

    @property
    def HTTP_PERMANENT_REDIRECT(self):
        """Message constant 'HTTP_PERMANENT_REDIRECT'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_PERMANENT_REDIRECT']

    @property
    def HTTP_BAD_REQUEST(self):
        """Message constant 'HTTP_BAD_REQUEST'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_BAD_REQUEST']

    @property
    def HTTP_UNAUTHORIZED(self):
        """Message constant 'HTTP_UNAUTHORIZED'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_UNAUTHORIZED']

    @property
    def HTTP_PAYMENT_REQUIRED(self):
        """Message constant 'HTTP_PAYMENT_REQUIRED'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_PAYMENT_REQUIRED']

    @property
    def HTTP_FORBIDDEN(self):
        """Message constant 'HTTP_FORBIDDEN'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_FORBIDDEN']

    @property
    def HTTP_NOT_FOUND(self):
        """Message constant 'HTTP_NOT_FOUND'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_NOT_FOUND']

    @property
    def HTTP_METHOD_NOT_ALLOWED(self):
        """Message constant 'HTTP_METHOD_NOT_ALLOWED'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_METHOD_NOT_ALLOWED']

    @property
    def HTTP_NOT_ACCEPTABLE(self):
        """Message constant 'HTTP_NOT_ACCEPTABLE'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_NOT_ACCEPTABLE']

    @property
    def HTTP_PROXY_AUTHENTICATION_REQUIRED(self):
        """Message constant 'HTTP_PROXY_AUTHENTICATION_REQUIRED'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_PROXY_AUTHENTICATION_REQUIRED']

    @property
    def HTTP_REQUEST_TIMEOUT(self):
        """Message constant 'HTTP_REQUEST_TIMEOUT'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_REQUEST_TIMEOUT']

    @property
    def HTTP_CONFLICT(self):
        """Message constant 'HTTP_CONFLICT'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_CONFLICT']

    @property
    def HTTP_GONE(self):
        """Message constant 'HTTP_GONE'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_GONE']

    @property
    def HTTP_LENGTH_REQUIRED(self):
        """Message constant 'HTTP_LENGTH_REQUIRED'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_LENGTH_REQUIRED']

    @property
    def HTTP_PRECONDITION_FAILED(self):
        """Message constant 'HTTP_PRECONDITION_FAILED'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_PRECONDITION_FAILED']

    @property
    def HTTP_REQUEST_ENTITY_TOO_LARGE(self):
        """Message constant 'HTTP_REQUEST_ENTITY_TOO_LARGE'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_REQUEST_ENTITY_TOO_LARGE']

    @property
    def HTTP_REQUEST_URI_TOOLONG(self):
        """Message constant 'HTTP_REQUEST_URI_TOOLONG'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_REQUEST_URI_TOOLONG']

    @property
    def HTTP_UNSUPPORTED_MEDIA_TYPE(self):
        """Message constant 'HTTP_UNSUPPORTED_MEDIA_TYPE'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_UNSUPPORTED_MEDIA_TYPE']

    @property
    def HTTP_REQUEST_RANGE_NOT_SATISFIABLE(self):
        """Message constant 'HTTP_REQUEST_RANGE_NOT_SATISFIABLE'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_REQUEST_RANGE_NOT_SATISFIABLE']

    @property
    def HTTP_EXPECTATION_FAILED(self):
        """Message constant 'HTTP_EXPECTATION_FAILED'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_EXPECTATION_FAILED']

    @property
    def HTTP_MISDIRECTED_REQUEST(self):
        """Message constant 'HTTP_MISDIRECTED_REQUEST'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_MISDIRECTED_REQUEST']

    @property
    def HTTP_UNPROCESSABLE_ENTITY(self):
        """Message constant 'HTTP_UNPROCESSABLE_ENTITY'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_UNPROCESSABLE_ENTITY']

    @property
    def HTTP_FAILED_DEPENDENCY(self):
        """Message constant 'HTTP_FAILED_DEPENDENCY'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_FAILED_DEPENDENCY']

    @property
    def HTTP_UPGRADE_REQUIRED(self):
        """Message constant 'HTTP_UPGRADE_REQUIRED'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_UPGRADE_REQUIRED']

    @property
    def HTTP_PRECONDITION_REQUIRED(self):
        """Message constant 'HTTP_PRECONDITION_REQUIRED'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_PRECONDITION_REQUIRED']

    @property
    def HTTP_TOO_MANY_REQUESTS(self):
        """Message constant 'HTTP_TOO_MANY_REQUESTS'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_TOO_MANY_REQUESTS']

    @property
    def HTTP_REQUEST_HEADER_FIELDS_TOO_LARGE(self):
        """Message constant 'HTTP_REQUEST_HEADER_FIELDS_TOO_LARGE'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_REQUEST_HEADER_FIELDS_TOO_LARGE']

    @property
    def HTTP_UNAVAILABLE_FOR_LEGAL_REASONS(self):
        """Message constant 'HTTP_UNAVAILABLE_FOR_LEGAL_REASONS'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_UNAVAILABLE_FOR_LEGAL_REASONS']

    @property
    def HTTP_INTERNAL_SERVER_ERROR(self):
        """Message constant 'HTTP_INTERNAL_SERVER_ERROR'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_INTERNAL_SERVER_ERROR']

    @property
    def HTTP_NOT_IMPLEMENTED(self):
        """Message constant 'HTTP_NOT_IMPLEMENTED'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_NOT_IMPLEMENTED']

    @property
    def HTTP_BAD_GATEWAY(self):
        """Message constant 'HTTP_BAD_GATEWAY'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_BAD_GATEWAY']

    @property
    def HTTP_SERVICE_UNAVAILABLE(self):
        """Message constant 'HTTP_SERVICE_UNAVAILABLE'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_SERVICE_UNAVAILABLE']

    @property
    def HTTP_GATEWAY_TIMEOUT(self):
        """Message constant 'HTTP_GATEWAY_TIMEOUT'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_GATEWAY_TIMEOUT']

    @property
    def HTTP_VERSION_NOT_SUPPORTED(self):
        """Message constant 'HTTP_VERSION_NOT_SUPPORTED'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_VERSION_NOT_SUPPORTED']

    @property
    def HTTP_VARIANT_ALSO_NEGOTIATES(self):
        """Message constant 'HTTP_VARIANT_ALSO_NEGOTIATES'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_VARIANT_ALSO_NEGOTIATES']

    @property
    def HTTP_INSUFFICIENT_STORAGE(self):
        """Message constant 'HTTP_INSUFFICIENT_STORAGE'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_INSUFFICIENT_STORAGE']

    @property
    def HTTP_NOT_EXTENDED(self):
        """Message constant 'HTTP_NOT_EXTENDED'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_NOT_EXTENDED']

    @property
    def HTTP_NETWORK_AUTHENTICATION_REQUIRED(self):
        """Message constant 'HTTP_NETWORK_AUTHENTICATION_REQUIRED'."""
        return Metaclass_HTTPStatusCode.__constants['HTTP_NETWORK_AUTHENTICATION_REQUIRED']


class HTTPStatusCode(metaclass=Metaclass_HTTPStatusCode):
    """
    Message class 'HTTPStatusCode'.

    Constants:
      HTTP_OK
      HTTP_ACCEPTED
      HTTP_NON_AUTHORITATIVE_INFORMATION
      HTTP_NO_CONTENT
      HTTP_RESET_CONTENT
      HTTP_PARTIAL_CONTENT
      HTTP_MULTIPLE_CHOICES
      HTTP_MOVED_PERMANENTLY
      HTTP_FOUND
      HTTP_SEE_OTHER
      HTTP_NOT_MODIFIED
      HTTP_USE_PROXY
      HTTP_TEMPORARY_REDIRECT
      HTTP_PERMANENT_REDIRECT
      HTTP_BAD_REQUEST
      HTTP_UNAUTHORIZED
      HTTP_PAYMENT_REQUIRED
      HTTP_FORBIDDEN
      HTTP_NOT_FOUND
      HTTP_METHOD_NOT_ALLOWED
      HTTP_NOT_ACCEPTABLE
      HTTP_PROXY_AUTHENTICATION_REQUIRED
      HTTP_REQUEST_TIMEOUT
      HTTP_CONFLICT
      HTTP_GONE
      HTTP_LENGTH_REQUIRED
      HTTP_PRECONDITION_FAILED
      HTTP_REQUEST_ENTITY_TOO_LARGE
      HTTP_REQUEST_URI_TOOLONG
      HTTP_UNSUPPORTED_MEDIA_TYPE
      HTTP_REQUEST_RANGE_NOT_SATISFIABLE
      HTTP_EXPECTATION_FAILED
      HTTP_MISDIRECTED_REQUEST
      HTTP_UNPROCESSABLE_ENTITY
      HTTP_FAILED_DEPENDENCY
      HTTP_UPGRADE_REQUIRED
      HTTP_PRECONDITION_REQUIRED
      HTTP_TOO_MANY_REQUESTS
      HTTP_REQUEST_HEADER_FIELDS_TOO_LARGE
      HTTP_UNAVAILABLE_FOR_LEGAL_REASONS
      HTTP_INTERNAL_SERVER_ERROR
      HTTP_NOT_IMPLEMENTED
      HTTP_BAD_GATEWAY
      HTTP_SERVICE_UNAVAILABLE
      HTTP_GATEWAY_TIMEOUT
      HTTP_VERSION_NOT_SUPPORTED
      HTTP_VARIANT_ALSO_NEGOTIATES
      HTTP_INSUFFICIENT_STORAGE
      HTTP_NOT_EXTENDED
      HTTP_NETWORK_AUTHENTICATION_REQUIRED
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
