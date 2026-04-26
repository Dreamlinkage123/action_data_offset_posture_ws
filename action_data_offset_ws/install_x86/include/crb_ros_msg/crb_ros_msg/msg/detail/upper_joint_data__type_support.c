// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from crb_ros_msg:msg/UpperJointData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "crb_ros_msg/msg/detail/upper_joint_data__rosidl_typesupport_introspection_c.h"
#include "crb_ros_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "crb_ros_msg/msg/detail/upper_joint_data__functions.h"
#include "crb_ros_msg/msg/detail/upper_joint_data__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `joint`
#include "sensor_msgs/msg/joint_state.h"
// Member `joint`
#include "sensor_msgs/msg/detail/joint_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void crb_ros_msg__msg__UpperJointData__rosidl_typesupport_introspection_c__UpperJointData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crb_ros_msg__msg__UpperJointData__init(message_memory);
}

void crb_ros_msg__msg__UpperJointData__rosidl_typesupport_introspection_c__UpperJointData_fini_function(void * message_memory)
{
  crb_ros_msg__msg__UpperJointData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember crb_ros_msg__msg__UpperJointData__rosidl_typesupport_introspection_c__UpperJointData_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crb_ros_msg__msg__UpperJointData, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_ref",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crb_ros_msg__msg__UpperJointData, time_ref),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vel_scale",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crb_ros_msg__msg__UpperJointData, vel_scale),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crb_ros_msg__msg__UpperJointData, joint),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crb_ros_msg__msg__UpperJointData__rosidl_typesupport_introspection_c__UpperJointData_message_members = {
  "crb_ros_msg__msg",  // message namespace
  "UpperJointData",  // message name
  4,  // number of fields
  sizeof(crb_ros_msg__msg__UpperJointData),
  crb_ros_msg__msg__UpperJointData__rosidl_typesupport_introspection_c__UpperJointData_message_member_array,  // message members
  crb_ros_msg__msg__UpperJointData__rosidl_typesupport_introspection_c__UpperJointData_init_function,  // function to initialize message memory (memory has to be allocated)
  crb_ros_msg__msg__UpperJointData__rosidl_typesupport_introspection_c__UpperJointData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crb_ros_msg__msg__UpperJointData__rosidl_typesupport_introspection_c__UpperJointData_message_type_support_handle = {
  0,
  &crb_ros_msg__msg__UpperJointData__rosidl_typesupport_introspection_c__UpperJointData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crb_ros_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crb_ros_msg, msg, UpperJointData)() {
  crb_ros_msg__msg__UpperJointData__rosidl_typesupport_introspection_c__UpperJointData_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  crb_ros_msg__msg__UpperJointData__rosidl_typesupport_introspection_c__UpperJointData_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, JointState)();
  if (!crb_ros_msg__msg__UpperJointData__rosidl_typesupport_introspection_c__UpperJointData_message_type_support_handle.typesupport_identifier) {
    crb_ros_msg__msg__UpperJointData__rosidl_typesupport_introspection_c__UpperJointData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crb_ros_msg__msg__UpperJointData__rosidl_typesupport_introspection_c__UpperJointData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
