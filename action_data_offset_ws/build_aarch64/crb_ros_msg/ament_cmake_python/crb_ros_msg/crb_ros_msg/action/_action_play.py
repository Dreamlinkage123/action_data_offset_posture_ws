# generated from rosidl_generator_py/resource/_idl.py.em
# with input from crb_ros_msg:action/ActionPlay.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ActionPlay_Goal(type):
    """Metaclass of message 'ActionPlay_Goal'."""

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
            module = import_type_support('crb_ros_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'crb_ros_msg.action.ActionPlay_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__action_play__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__action_play__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__action_play__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__action_play__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__action_play__goal

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ActionPlay_Goal(metaclass=Metaclass_ActionPlay_Goal):
    """Message class 'ActionPlay_Goal'."""

    __slots__ = [
        '_start_time',
        '_action_name',
        '_cancel_action_name',
        '_rl_name',
    ]

    _fields_and_field_types = {
        'start_time': 'double',
        'action_name': 'string',
        'cancel_action_name': 'string',
        'rl_name': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.start_time = kwargs.get('start_time', float())
        self.action_name = kwargs.get('action_name', str())
        self.cancel_action_name = kwargs.get('cancel_action_name', str())
        self.rl_name = kwargs.get('rl_name', str())

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
        if self.start_time != other.start_time:
            return False
        if self.action_name != other.action_name:
            return False
        if self.cancel_action_name != other.cancel_action_name:
            return False
        if self.rl_name != other.rl_name:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def start_time(self):
        """Message field 'start_time'."""
        return self._start_time

    @start_time.setter
    def start_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'start_time' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'start_time' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._start_time = value

    @builtins.property
    def action_name(self):
        """Message field 'action_name'."""
        return self._action_name

    @action_name.setter
    def action_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'action_name' field must be of type 'str'"
        self._action_name = value

    @builtins.property
    def cancel_action_name(self):
        """Message field 'cancel_action_name'."""
        return self._cancel_action_name

    @cancel_action_name.setter
    def cancel_action_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'cancel_action_name' field must be of type 'str'"
        self._cancel_action_name = value

    @builtins.property
    def rl_name(self):
        """Message field 'rl_name'."""
        return self._rl_name

    @rl_name.setter
    def rl_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'rl_name' field must be of type 'str'"
        self._rl_name = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ActionPlay_Result(type):
    """Metaclass of message 'ActionPlay_Result'."""

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
            module = import_type_support('crb_ros_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'crb_ros_msg.action.ActionPlay_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__action_play__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__action_play__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__action_play__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__action_play__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__action_play__result

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ActionPlay_Result(metaclass=Metaclass_ActionPlay_Result):
    """Message class 'ActionPlay_Result'."""

    __slots__ = [
        '_if_success',
    ]

    _fields_and_field_types = {
        'if_success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.if_success = kwargs.get('if_success', bool())

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
        if self.if_success != other.if_success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def if_success(self):
        """Message field 'if_success'."""
        return self._if_success

    @if_success.setter
    def if_success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'if_success' field must be of type 'bool'"
        self._if_success = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_ActionPlay_Feedback(type):
    """Metaclass of message 'ActionPlay_Feedback'."""

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
            module = import_type_support('crb_ros_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'crb_ros_msg.action.ActionPlay_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__action_play__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__action_play__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__action_play__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__action_play__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__action_play__feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ActionPlay_Feedback(metaclass=Metaclass_ActionPlay_Feedback):
    """Message class 'ActionPlay_Feedback'."""

    __slots__ = [
        '_action_index',
        '_exec_time',
        '_state',
    ]

    _fields_and_field_types = {
        'action_index': 'uint64',
        'exec_time': 'double',
        'state': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.action_index = kwargs.get('action_index', int())
        self.exec_time = kwargs.get('exec_time', float())
        self.state = kwargs.get('state', int())

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
        if self.action_index != other.action_index:
            return False
        if self.exec_time != other.exec_time:
            return False
        if self.state != other.state:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def action_index(self):
        """Message field 'action_index'."""
        return self._action_index

    @action_index.setter
    def action_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'action_index' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'action_index' field must be an unsigned integer in [0, 18446744073709551615]"
        self._action_index = value

    @builtins.property
    def exec_time(self):
        """Message field 'exec_time'."""
        return self._exec_time

    @exec_time.setter
    def exec_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'exec_time' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'exec_time' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._exec_time = value

    @builtins.property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'state' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'state' field must be an integer in [-2147483648, 2147483647]"
        self._state = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ActionPlay_SendGoal_Request(type):
    """Metaclass of message 'ActionPlay_SendGoal_Request'."""

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
            module = import_type_support('crb_ros_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'crb_ros_msg.action.ActionPlay_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__action_play__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__action_play__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__action_play__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__action_play__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__action_play__send_goal__request

            from crb_ros_msg.action import ActionPlay
            if ActionPlay.Goal.__class__._TYPE_SUPPORT is None:
                ActionPlay.Goal.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ActionPlay_SendGoal_Request(metaclass=Metaclass_ActionPlay_SendGoal_Request):
    """Message class 'ActionPlay_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'crb_ros_msg/ActionPlay_Goal',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['crb_ros_msg', 'action'], 'ActionPlay_Goal'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from crb_ros_msg.action._action_play import ActionPlay_Goal
        self.goal = kwargs.get('goal', ActionPlay_Goal())

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
        if self.goal_id != other.goal_id:
            return False
        if self.goal != other.goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            from crb_ros_msg.action._action_play import ActionPlay_Goal
            assert \
                isinstance(value, ActionPlay_Goal), \
                "The 'goal' field must be a sub message of type 'ActionPlay_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ActionPlay_SendGoal_Response(type):
    """Metaclass of message 'ActionPlay_SendGoal_Response'."""

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
            module = import_type_support('crb_ros_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'crb_ros_msg.action.ActionPlay_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__action_play__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__action_play__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__action_play__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__action_play__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__action_play__send_goal__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ActionPlay_SendGoal_Response(metaclass=Metaclass_ActionPlay_SendGoal_Response):
    """Message class 'ActionPlay_SendGoal_Response'."""

    __slots__ = [
        '_accepted',
        '_stamp',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'stamp': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())

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
        if self.accepted != other.accepted:
            return False
        if self.stamp != other.stamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value


class Metaclass_ActionPlay_SendGoal(type):
    """Metaclass of service 'ActionPlay_SendGoal'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('crb_ros_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'crb_ros_msg.action.ActionPlay_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__action_play__send_goal

            from crb_ros_msg.action import _action_play
            if _action_play.Metaclass_ActionPlay_SendGoal_Request._TYPE_SUPPORT is None:
                _action_play.Metaclass_ActionPlay_SendGoal_Request.__import_type_support__()
            if _action_play.Metaclass_ActionPlay_SendGoal_Response._TYPE_SUPPORT is None:
                _action_play.Metaclass_ActionPlay_SendGoal_Response.__import_type_support__()


class ActionPlay_SendGoal(metaclass=Metaclass_ActionPlay_SendGoal):
    from crb_ros_msg.action._action_play import ActionPlay_SendGoal_Request as Request
    from crb_ros_msg.action._action_play import ActionPlay_SendGoal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ActionPlay_GetResult_Request(type):
    """Metaclass of message 'ActionPlay_GetResult_Request'."""

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
            module = import_type_support('crb_ros_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'crb_ros_msg.action.ActionPlay_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__action_play__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__action_play__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__action_play__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__action_play__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__action_play__get_result__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ActionPlay_GetResult_Request(metaclass=Metaclass_ActionPlay_GetResult_Request):
    """Message class 'ActionPlay_GetResult_Request'."""

    __slots__ = [
        '_goal_id',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())

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
        if self.goal_id != other.goal_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ActionPlay_GetResult_Response(type):
    """Metaclass of message 'ActionPlay_GetResult_Response'."""

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
            module = import_type_support('crb_ros_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'crb_ros_msg.action.ActionPlay_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__action_play__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__action_play__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__action_play__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__action_play__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__action_play__get_result__response

            from crb_ros_msg.action import ActionPlay
            if ActionPlay.Result.__class__._TYPE_SUPPORT is None:
                ActionPlay.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ActionPlay_GetResult_Response(metaclass=Metaclass_ActionPlay_GetResult_Response):
    """Message class 'ActionPlay_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'crb_ros_msg/ActionPlay_Result',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['crb_ros_msg', 'action'], 'ActionPlay_Result'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        from crb_ros_msg.action._action_play import ActionPlay_Result
        self.result = kwargs.get('result', ActionPlay_Result())

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
        if self.result != other.result:
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
            assert value >= -128 and value < 128, \
                "The 'status' field must be an integer in [-128, 127]"
        self._status = value

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            from crb_ros_msg.action._action_play import ActionPlay_Result
            assert \
                isinstance(value, ActionPlay_Result), \
                "The 'result' field must be a sub message of type 'ActionPlay_Result'"
        self._result = value


class Metaclass_ActionPlay_GetResult(type):
    """Metaclass of service 'ActionPlay_GetResult'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('crb_ros_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'crb_ros_msg.action.ActionPlay_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__action_play__get_result

            from crb_ros_msg.action import _action_play
            if _action_play.Metaclass_ActionPlay_GetResult_Request._TYPE_SUPPORT is None:
                _action_play.Metaclass_ActionPlay_GetResult_Request.__import_type_support__()
            if _action_play.Metaclass_ActionPlay_GetResult_Response._TYPE_SUPPORT is None:
                _action_play.Metaclass_ActionPlay_GetResult_Response.__import_type_support__()


class ActionPlay_GetResult(metaclass=Metaclass_ActionPlay_GetResult):
    from crb_ros_msg.action._action_play import ActionPlay_GetResult_Request as Request
    from crb_ros_msg.action._action_play import ActionPlay_GetResult_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ActionPlay_FeedbackMessage(type):
    """Metaclass of message 'ActionPlay_FeedbackMessage'."""

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
            module = import_type_support('crb_ros_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'crb_ros_msg.action.ActionPlay_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__action_play__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__action_play__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__action_play__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__action_play__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__action_play__feedback_message

            from crb_ros_msg.action import ActionPlay
            if ActionPlay.Feedback.__class__._TYPE_SUPPORT is None:
                ActionPlay.Feedback.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ActionPlay_FeedbackMessage(metaclass=Metaclass_ActionPlay_FeedbackMessage):
    """Message class 'ActionPlay_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'crb_ros_msg/ActionPlay_Feedback',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['crb_ros_msg', 'action'], 'ActionPlay_Feedback'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from crb_ros_msg.action._action_play import ActionPlay_Feedback
        self.feedback = kwargs.get('feedback', ActionPlay_Feedback())

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
        if self.goal_id != other.goal_id:
            return False
        if self.feedback != other.feedback:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def feedback(self):
        """Message field 'feedback'."""
        return self._feedback

    @feedback.setter
    def feedback(self, value):
        if __debug__:
            from crb_ros_msg.action._action_play import ActionPlay_Feedback
            assert \
                isinstance(value, ActionPlay_Feedback), \
                "The 'feedback' field must be a sub message of type 'ActionPlay_Feedback'"
        self._feedback = value


class Metaclass_ActionPlay(type):
    """Metaclass of action 'ActionPlay'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('crb_ros_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'crb_ros_msg.action.ActionPlay')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__action_play

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from crb_ros_msg.action import _action_play
            if _action_play.Metaclass_ActionPlay_SendGoal._TYPE_SUPPORT is None:
                _action_play.Metaclass_ActionPlay_SendGoal.__import_type_support__()
            if _action_play.Metaclass_ActionPlay_GetResult._TYPE_SUPPORT is None:
                _action_play.Metaclass_ActionPlay_GetResult.__import_type_support__()
            if _action_play.Metaclass_ActionPlay_FeedbackMessage._TYPE_SUPPORT is None:
                _action_play.Metaclass_ActionPlay_FeedbackMessage.__import_type_support__()


class ActionPlay(metaclass=Metaclass_ActionPlay):

    # The goal message defined in the action definition.
    from crb_ros_msg.action._action_play import ActionPlay_Goal as Goal
    # The result message defined in the action definition.
    from crb_ros_msg.action._action_play import ActionPlay_Result as Result
    # The feedback message defined in the action definition.
    from crb_ros_msg.action._action_play import ActionPlay_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from crb_ros_msg.action._action_play import ActionPlay_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from crb_ros_msg.action._action_play import ActionPlay_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from crb_ros_msg.action._action_play import ActionPlay_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')
