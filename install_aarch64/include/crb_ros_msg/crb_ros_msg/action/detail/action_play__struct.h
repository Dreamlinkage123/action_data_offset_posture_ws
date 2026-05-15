// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crb_ros_msg:action/ActionPlay.idl
// generated code does not contain a copyright notice

#ifndef CRB_ROS_MSG__ACTION__DETAIL__ACTION_PLAY__STRUCT_H_
#define CRB_ROS_MSG__ACTION__DETAIL__ACTION_PLAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'action_name'
// Member 'cancel_action_name'
// Member 'rl_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/ActionPlay in the package crb_ros_msg.
typedef struct crb_ros_msg__action__ActionPlay_Goal
{
  double start_time;
  rosidl_runtime_c__String action_name;
  rosidl_runtime_c__String cancel_action_name;
  rosidl_runtime_c__String rl_name;
} crb_ros_msg__action__ActionPlay_Goal;

// Struct for a sequence of crb_ros_msg__action__ActionPlay_Goal.
typedef struct crb_ros_msg__action__ActionPlay_Goal__Sequence
{
  crb_ros_msg__action__ActionPlay_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crb_ros_msg__action__ActionPlay_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/ActionPlay in the package crb_ros_msg.
typedef struct crb_ros_msg__action__ActionPlay_Result
{
  bool if_success;
} crb_ros_msg__action__ActionPlay_Result;

// Struct for a sequence of crb_ros_msg__action__ActionPlay_Result.
typedef struct crb_ros_msg__action__ActionPlay_Result__Sequence
{
  crb_ros_msg__action__ActionPlay_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crb_ros_msg__action__ActionPlay_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/ActionPlay in the package crb_ros_msg.
typedef struct crb_ros_msg__action__ActionPlay_Feedback
{
  uint64_t action_index;
  double exec_time;
  /// 状态，0：未播放，1：加载文件中；2：动作播放中；3：动作结束; 4: 动作取消； 5： 动作取消结束
  int32_t state;
} crb_ros_msg__action__ActionPlay_Feedback;

// Struct for a sequence of crb_ros_msg__action__ActionPlay_Feedback.
typedef struct crb_ros_msg__action__ActionPlay_Feedback__Sequence
{
  crb_ros_msg__action__ActionPlay_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crb_ros_msg__action__ActionPlay_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "crb_ros_msg/action/detail/action_play__struct.h"

/// Struct defined in action/ActionPlay in the package crb_ros_msg.
typedef struct crb_ros_msg__action__ActionPlay_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  crb_ros_msg__action__ActionPlay_Goal goal;
} crb_ros_msg__action__ActionPlay_SendGoal_Request;

// Struct for a sequence of crb_ros_msg__action__ActionPlay_SendGoal_Request.
typedef struct crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence
{
  crb_ros_msg__action__ActionPlay_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/ActionPlay in the package crb_ros_msg.
typedef struct crb_ros_msg__action__ActionPlay_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} crb_ros_msg__action__ActionPlay_SendGoal_Response;

// Struct for a sequence of crb_ros_msg__action__ActionPlay_SendGoal_Response.
typedef struct crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence
{
  crb_ros_msg__action__ActionPlay_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/ActionPlay in the package crb_ros_msg.
typedef struct crb_ros_msg__action__ActionPlay_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} crb_ros_msg__action__ActionPlay_GetResult_Request;

// Struct for a sequence of crb_ros_msg__action__ActionPlay_GetResult_Request.
typedef struct crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence
{
  crb_ros_msg__action__ActionPlay_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "crb_ros_msg/action/detail/action_play__struct.h"

/// Struct defined in action/ActionPlay in the package crb_ros_msg.
typedef struct crb_ros_msg__action__ActionPlay_GetResult_Response
{
  int8_t status;
  crb_ros_msg__action__ActionPlay_Result result;
} crb_ros_msg__action__ActionPlay_GetResult_Response;

// Struct for a sequence of crb_ros_msg__action__ActionPlay_GetResult_Response.
typedef struct crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence
{
  crb_ros_msg__action__ActionPlay_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "crb_ros_msg/action/detail/action_play__struct.h"

/// Struct defined in action/ActionPlay in the package crb_ros_msg.
typedef struct crb_ros_msg__action__ActionPlay_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  crb_ros_msg__action__ActionPlay_Feedback feedback;
} crb_ros_msg__action__ActionPlay_FeedbackMessage;

// Struct for a sequence of crb_ros_msg__action__ActionPlay_FeedbackMessage.
typedef struct crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence
{
  crb_ros_msg__action__ActionPlay_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRB_ROS_MSG__ACTION__DETAIL__ACTION_PLAY__STRUCT_H_
