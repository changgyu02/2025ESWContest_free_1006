# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros2web_interfaces:msg/ContentType.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ContentType(type):
    """Metaclass of message 'ContentType'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'APPLICATION_JSON': 'application/json',
        'APPLICATION_MSWORD': 'application/msword',
        'APPLICATION_MSWORD_OPEN_XML': 'application/vnd.openxmlformats-officedocument.wordprocessingml.document',
        'APPLICATION_OCTET_STREAM': 'application/octet-stream',
        'APPLICATION_VND_AMAZON_EBOOK': 'application/vnd.amazon.ebook',
        'APPLICATION_X_ABIWORD': 'application/x-abiword',
        'APPLICATION_X_BZIP2': 'application/x-bzip2',
        'APPLICATION_X_BZIP': 'application/x-bzip',
        'APPLICATION_X_CSH': 'application/x-csh',
        'APPLICATION_X_FREEARC': 'application/x-freearc',
        'APPLICATION_X_WWW_FORM_URLENCODED': 'application/x-www-form-urlencoded',
        'AUDIO_AAC': 'audio/aac',
        'IMAGE_BMP': 'image/bmp',
        'IMAGE_JPEG': 'image/jpeg',
        'IMAGE_PNG': 'image/png',
        'MULTIPART_FORM_DATA': 'multipart/form-data',
        'TEXT_CSS': 'text/css',
        'TEXT_CSV': 'text/csv',
        'TEXT_HTML': 'text/html',
        'TEXT_JAVASCRIPT': 'text/javascript',
        'TEXT_PLAIN': 'text/plain',
        'TEXT_XML': 'text/xml',
        'VIDEO_X_MSVIDEO': 'video/x-msvideo',
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
                'ros2web_interfaces.msg.ContentType')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__content_type
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__content_type
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__content_type
            cls._TYPE_SUPPORT = module.type_support_msg__msg__content_type
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__content_type

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'APPLICATION_JSON': cls.__constants['APPLICATION_JSON'],
            'APPLICATION_MSWORD': cls.__constants['APPLICATION_MSWORD'],
            'APPLICATION_MSWORD_OPEN_XML': cls.__constants['APPLICATION_MSWORD_OPEN_XML'],
            'APPLICATION_OCTET_STREAM': cls.__constants['APPLICATION_OCTET_STREAM'],
            'APPLICATION_VND_AMAZON_EBOOK': cls.__constants['APPLICATION_VND_AMAZON_EBOOK'],
            'APPLICATION_X_ABIWORD': cls.__constants['APPLICATION_X_ABIWORD'],
            'APPLICATION_X_BZIP2': cls.__constants['APPLICATION_X_BZIP2'],
            'APPLICATION_X_BZIP': cls.__constants['APPLICATION_X_BZIP'],
            'APPLICATION_X_CSH': cls.__constants['APPLICATION_X_CSH'],
            'APPLICATION_X_FREEARC': cls.__constants['APPLICATION_X_FREEARC'],
            'APPLICATION_X_WWW_FORM_URLENCODED': cls.__constants['APPLICATION_X_WWW_FORM_URLENCODED'],
            'AUDIO_AAC': cls.__constants['AUDIO_AAC'],
            'IMAGE_BMP': cls.__constants['IMAGE_BMP'],
            'IMAGE_JPEG': cls.__constants['IMAGE_JPEG'],
            'IMAGE_PNG': cls.__constants['IMAGE_PNG'],
            'MULTIPART_FORM_DATA': cls.__constants['MULTIPART_FORM_DATA'],
            'TEXT_CSS': cls.__constants['TEXT_CSS'],
            'TEXT_CSV': cls.__constants['TEXT_CSV'],
            'TEXT_HTML': cls.__constants['TEXT_HTML'],
            'TEXT_JAVASCRIPT': cls.__constants['TEXT_JAVASCRIPT'],
            'TEXT_PLAIN': cls.__constants['TEXT_PLAIN'],
            'TEXT_XML': cls.__constants['TEXT_XML'],
            'VIDEO_X_MSVIDEO': cls.__constants['VIDEO_X_MSVIDEO'],
        }

    @property
    def APPLICATION_JSON(self):
        """Message constant 'APPLICATION_JSON'."""
        return Metaclass_ContentType.__constants['APPLICATION_JSON']

    @property
    def APPLICATION_MSWORD(self):
        """Message constant 'APPLICATION_MSWORD'."""
        return Metaclass_ContentType.__constants['APPLICATION_MSWORD']

    @property
    def APPLICATION_MSWORD_OPEN_XML(self):
        """Message constant 'APPLICATION_MSWORD_OPEN_XML'."""
        return Metaclass_ContentType.__constants['APPLICATION_MSWORD_OPEN_XML']

    @property
    def APPLICATION_OCTET_STREAM(self):
        """Message constant 'APPLICATION_OCTET_STREAM'."""
        return Metaclass_ContentType.__constants['APPLICATION_OCTET_STREAM']

    @property
    def APPLICATION_VND_AMAZON_EBOOK(self):
        """Message constant 'APPLICATION_VND_AMAZON_EBOOK'."""
        return Metaclass_ContentType.__constants['APPLICATION_VND_AMAZON_EBOOK']

    @property
    def APPLICATION_X_ABIWORD(self):
        """Message constant 'APPLICATION_X_ABIWORD'."""
        return Metaclass_ContentType.__constants['APPLICATION_X_ABIWORD']

    @property
    def APPLICATION_X_BZIP2(self):
        """Message constant 'APPLICATION_X_BZIP2'."""
        return Metaclass_ContentType.__constants['APPLICATION_X_BZIP2']

    @property
    def APPLICATION_X_BZIP(self):
        """Message constant 'APPLICATION_X_BZIP'."""
        return Metaclass_ContentType.__constants['APPLICATION_X_BZIP']

    @property
    def APPLICATION_X_CSH(self):
        """Message constant 'APPLICATION_X_CSH'."""
        return Metaclass_ContentType.__constants['APPLICATION_X_CSH']

    @property
    def APPLICATION_X_FREEARC(self):
        """Message constant 'APPLICATION_X_FREEARC'."""
        return Metaclass_ContentType.__constants['APPLICATION_X_FREEARC']

    @property
    def APPLICATION_X_WWW_FORM_URLENCODED(self):
        """Message constant 'APPLICATION_X_WWW_FORM_URLENCODED'."""
        return Metaclass_ContentType.__constants['APPLICATION_X_WWW_FORM_URLENCODED']

    @property
    def AUDIO_AAC(self):
        """Message constant 'AUDIO_AAC'."""
        return Metaclass_ContentType.__constants['AUDIO_AAC']

    @property
    def IMAGE_BMP(self):
        """Message constant 'IMAGE_BMP'."""
        return Metaclass_ContentType.__constants['IMAGE_BMP']

    @property
    def IMAGE_JPEG(self):
        """Message constant 'IMAGE_JPEG'."""
        return Metaclass_ContentType.__constants['IMAGE_JPEG']

    @property
    def IMAGE_PNG(self):
        """Message constant 'IMAGE_PNG'."""
        return Metaclass_ContentType.__constants['IMAGE_PNG']

    @property
    def MULTIPART_FORM_DATA(self):
        """Message constant 'MULTIPART_FORM_DATA'."""
        return Metaclass_ContentType.__constants['MULTIPART_FORM_DATA']

    @property
    def TEXT_CSS(self):
        """Message constant 'TEXT_CSS'."""
        return Metaclass_ContentType.__constants['TEXT_CSS']

    @property
    def TEXT_CSV(self):
        """Message constant 'TEXT_CSV'."""
        return Metaclass_ContentType.__constants['TEXT_CSV']

    @property
    def TEXT_HTML(self):
        """Message constant 'TEXT_HTML'."""
        return Metaclass_ContentType.__constants['TEXT_HTML']

    @property
    def TEXT_JAVASCRIPT(self):
        """Message constant 'TEXT_JAVASCRIPT'."""
        return Metaclass_ContentType.__constants['TEXT_JAVASCRIPT']

    @property
    def TEXT_PLAIN(self):
        """Message constant 'TEXT_PLAIN'."""
        return Metaclass_ContentType.__constants['TEXT_PLAIN']

    @property
    def TEXT_XML(self):
        """Message constant 'TEXT_XML'."""
        return Metaclass_ContentType.__constants['TEXT_XML']

    @property
    def VIDEO_X_MSVIDEO(self):
        """Message constant 'VIDEO_X_MSVIDEO'."""
        return Metaclass_ContentType.__constants['VIDEO_X_MSVIDEO']


class ContentType(metaclass=Metaclass_ContentType):
    """
    Message class 'ContentType'.

    Constants:
      APPLICATION_JSON
      APPLICATION_MSWORD
      APPLICATION_MSWORD_OPEN_XML
      APPLICATION_OCTET_STREAM
      APPLICATION_VND_AMAZON_EBOOK
      APPLICATION_X_ABIWORD
      APPLICATION_X_BZIP2
      APPLICATION_X_BZIP
      APPLICATION_X_CSH
      APPLICATION_X_FREEARC
      APPLICATION_X_WWW_FORM_URLENCODED
      AUDIO_AAC
      IMAGE_BMP
      IMAGE_JPEG
      IMAGE_PNG
      MULTIPART_FORM_DATA
      TEXT_CSS
      TEXT_CSV
      TEXT_HTML
      TEXT_JAVASCRIPT
      TEXT_PLAIN
      TEXT_XML
      VIDEO_X_MSVIDEO
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
