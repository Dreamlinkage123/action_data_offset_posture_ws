// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crb_ros_msg:msg/UpperJointData.idl
// generated code does not contain a copyright notice

#ifndef CRB_ROS_MSG__MSG__DETAIL__UPPER_JOINT_DATA__STRUCT_H_
#define CRB_ROS_MSG__MSG__DETAIL__UPPER_JOINT_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'joint'
#include "sensor_msgs/msg/detail/joint_state__struct.h"

/// Struct defined in msg/UpperJointData in the package crb_ros_msg.
typedef struct crb_ros_msg__msg__UpperJointData
{
  std_msgs__msg__Header header;
  float time_ref;
  float vel_scale;
  sensor_msgs__msg__JointState joint;
} crb_ros_msg__msg__UpperJointData;

// Struct for a sequence of crb_ros_msg__msg__UpperJointData.
typedef struct crb_ros_msg__msg__UpperJointData__Sequence
{
  crb_ros_msg__msg__UpperJointData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crb_ros_msg__msg__UpperJointData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRB_ROS_MSG__MSG__DETAIL__UPPER_JOINT_DATA__STRUCT_H_
