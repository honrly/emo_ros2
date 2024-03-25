# generated from rosidl_generator_py/resource/_idl.py.em
# with input from emo_voice_interfaces:srv/GenText.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GenText_Request(type):
    """Metaclass of message 'GenText_Request'."""

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
            module = import_type_support('emo_voice_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'emo_voice_interfaces.srv.GenText_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__gen_text__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__gen_text__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__gen_text__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__gen_text__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__gen_text__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GenText_Request(metaclass=Metaclass_GenText_Request):
    """Message class 'GenText_Request'."""

    __slots__ = [
        '_emo',
        '_name',
    ]

    _fields_and_field_types = {
        'emo': 'string',
        'name': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.emo = kwargs.get('emo', str())
        self.name = kwargs.get('name', str())

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
        if self.emo != other.emo:
            return False
        if self.name != other.name:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def emo(self):
        """Message field 'emo'."""
        return self._emo

    @emo.setter
    def emo(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'emo' field must be of type 'str'"
        self._emo = value

    @builtins.property
    def name(self):
        """Message field 'name'."""
        return self._name

    @name.setter
    def name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'name' field must be of type 'str'"
        self._name = value


# Import statements for member types

# Member 'text'
import array  # noqa: E402, I100

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GenText_Response(type):
    """Metaclass of message 'GenText_Response'."""

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
            module = import_type_support('emo_voice_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'emo_voice_interfaces.srv.GenText_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__gen_text__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__gen_text__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__gen_text__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__gen_text__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__gen_text__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GenText_Response(metaclass=Metaclass_GenText_Response):
    """Message class 'GenText_Response'."""

    __slots__ = [
        '_text',
    ]

    _fields_and_field_types = {
        'text': 'sequence<int32>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.text = array.array('i', kwargs.get('text', []))

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
        if self.text != other.text:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def text(self):
        """Message field 'text'."""
        return self._text

    @text.setter
    def text(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'text' array.array() must have the type code of 'i'"
            self._text = value
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
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'text' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._text = array.array('i', value)


class Metaclass_GenText(type):
    """Metaclass of service 'GenText'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('emo_voice_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'emo_voice_interfaces.srv.GenText')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__gen_text

            from emo_voice_interfaces.srv import _gen_text
            if _gen_text.Metaclass_GenText_Request._TYPE_SUPPORT is None:
                _gen_text.Metaclass_GenText_Request.__import_type_support__()
            if _gen_text.Metaclass_GenText_Response._TYPE_SUPPORT is None:
                _gen_text.Metaclass_GenText_Response.__import_type_support__()


class GenText(metaclass=Metaclass_GenText):
    from emo_voice_interfaces.srv._gen_text import GenText_Request as Request
    from emo_voice_interfaces.srv._gen_text import GenText_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
