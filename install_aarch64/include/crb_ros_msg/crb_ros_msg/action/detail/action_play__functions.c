// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from crb_ros_msg:action/ActionPlay.idl
// generated code does not contain a copyright notice
#include "crb_ros_msg/action/detail/action_play__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `action_name`
// Member `cancel_action_name`
// Member `rl_name`
#include "rosidl_runtime_c/string_functions.h"

bool
crb_ros_msg__action__ActionPlay_Goal__init(crb_ros_msg__action__ActionPlay_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // start_time
  // action_name
  if (!rosidl_runtime_c__String__init(&msg->action_name)) {
    crb_ros_msg__action__ActionPlay_Goal__fini(msg);
    return false;
  }
  // cancel_action_name
  if (!rosidl_runtime_c__String__init(&msg->cancel_action_name)) {
    crb_ros_msg__action__ActionPlay_Goal__fini(msg);
    return false;
  }
  // rl_name
  if (!rosidl_runtime_c__String__init(&msg->rl_name)) {
    crb_ros_msg__action__ActionPlay_Goal__fini(msg);
    return false;
  }
  return true;
}

void
crb_ros_msg__action__ActionPlay_Goal__fini(crb_ros_msg__action__ActionPlay_Goal * msg)
{
  if (!msg) {
    return;
  }
  // start_time
  // action_name
  rosidl_runtime_c__String__fini(&msg->action_name);
  // cancel_action_name
  rosidl_runtime_c__String__fini(&msg->cancel_action_name);
  // rl_name
  rosidl_runtime_c__String__fini(&msg->rl_name);
}

bool
crb_ros_msg__action__ActionPlay_Goal__are_equal(const crb_ros_msg__action__ActionPlay_Goal * lhs, const crb_ros_msg__action__ActionPlay_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // start_time
  if (lhs->start_time != rhs->start_time) {
    return false;
  }
  // action_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->action_name), &(rhs->action_name)))
  {
    return false;
  }
  // cancel_action_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->cancel_action_name), &(rhs->cancel_action_name)))
  {
    return false;
  }
  // rl_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->rl_name), &(rhs->rl_name)))
  {
    return false;
  }
  return true;
}

bool
crb_ros_msg__action__ActionPlay_Goal__copy(
  const crb_ros_msg__action__ActionPlay_Goal * input,
  crb_ros_msg__action__ActionPlay_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // start_time
  output->start_time = input->start_time;
  // action_name
  if (!rosidl_runtime_c__String__copy(
      &(input->action_name), &(output->action_name)))
  {
    return false;
  }
  // cancel_action_name
  if (!rosidl_runtime_c__String__copy(
      &(input->cancel_action_name), &(output->cancel_action_name)))
  {
    return false;
  }
  // rl_name
  if (!rosidl_runtime_c__String__copy(
      &(input->rl_name), &(output->rl_name)))
  {
    return false;
  }
  return true;
}

crb_ros_msg__action__ActionPlay_Goal *
crb_ros_msg__action__ActionPlay_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_Goal * msg = (crb_ros_msg__action__ActionPlay_Goal *)allocator.allocate(sizeof(crb_ros_msg__action__ActionPlay_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crb_ros_msg__action__ActionPlay_Goal));
  bool success = crb_ros_msg__action__ActionPlay_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crb_ros_msg__action__ActionPlay_Goal__destroy(crb_ros_msg__action__ActionPlay_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crb_ros_msg__action__ActionPlay_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crb_ros_msg__action__ActionPlay_Goal__Sequence__init(crb_ros_msg__action__ActionPlay_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_Goal * data = NULL;

  if (size) {
    data = (crb_ros_msg__action__ActionPlay_Goal *)allocator.zero_allocate(size, sizeof(crb_ros_msg__action__ActionPlay_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crb_ros_msg__action__ActionPlay_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crb_ros_msg__action__ActionPlay_Goal__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
crb_ros_msg__action__ActionPlay_Goal__Sequence__fini(crb_ros_msg__action__ActionPlay_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      crb_ros_msg__action__ActionPlay_Goal__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

crb_ros_msg__action__ActionPlay_Goal__Sequence *
crb_ros_msg__action__ActionPlay_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_Goal__Sequence * array = (crb_ros_msg__action__ActionPlay_Goal__Sequence *)allocator.allocate(sizeof(crb_ros_msg__action__ActionPlay_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crb_ros_msg__action__ActionPlay_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crb_ros_msg__action__ActionPlay_Goal__Sequence__destroy(crb_ros_msg__action__ActionPlay_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crb_ros_msg__action__ActionPlay_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crb_ros_msg__action__ActionPlay_Goal__Sequence__are_equal(const crb_ros_msg__action__ActionPlay_Goal__Sequence * lhs, const crb_ros_msg__action__ActionPlay_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crb_ros_msg__action__ActionPlay_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crb_ros_msg__action__ActionPlay_Goal__Sequence__copy(
  const crb_ros_msg__action__ActionPlay_Goal__Sequence * input,
  crb_ros_msg__action__ActionPlay_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crb_ros_msg__action__ActionPlay_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crb_ros_msg__action__ActionPlay_Goal * data =
      (crb_ros_msg__action__ActionPlay_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crb_ros_msg__action__ActionPlay_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crb_ros_msg__action__ActionPlay_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crb_ros_msg__action__ActionPlay_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
crb_ros_msg__action__ActionPlay_Result__init(crb_ros_msg__action__ActionPlay_Result * msg)
{
  if (!msg) {
    return false;
  }
  // if_success
  return true;
}

void
crb_ros_msg__action__ActionPlay_Result__fini(crb_ros_msg__action__ActionPlay_Result * msg)
{
  if (!msg) {
    return;
  }
  // if_success
}

bool
crb_ros_msg__action__ActionPlay_Result__are_equal(const crb_ros_msg__action__ActionPlay_Result * lhs, const crb_ros_msg__action__ActionPlay_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // if_success
  if (lhs->if_success != rhs->if_success) {
    return false;
  }
  return true;
}

bool
crb_ros_msg__action__ActionPlay_Result__copy(
  const crb_ros_msg__action__ActionPlay_Result * input,
  crb_ros_msg__action__ActionPlay_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // if_success
  output->if_success = input->if_success;
  return true;
}

crb_ros_msg__action__ActionPlay_Result *
crb_ros_msg__action__ActionPlay_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_Result * msg = (crb_ros_msg__action__ActionPlay_Result *)allocator.allocate(sizeof(crb_ros_msg__action__ActionPlay_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crb_ros_msg__action__ActionPlay_Result));
  bool success = crb_ros_msg__action__ActionPlay_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crb_ros_msg__action__ActionPlay_Result__destroy(crb_ros_msg__action__ActionPlay_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crb_ros_msg__action__ActionPlay_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crb_ros_msg__action__ActionPlay_Result__Sequence__init(crb_ros_msg__action__ActionPlay_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_Result * data = NULL;

  if (size) {
    data = (crb_ros_msg__action__ActionPlay_Result *)allocator.zero_allocate(size, sizeof(crb_ros_msg__action__ActionPlay_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crb_ros_msg__action__ActionPlay_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crb_ros_msg__action__ActionPlay_Result__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
crb_ros_msg__action__ActionPlay_Result__Sequence__fini(crb_ros_msg__action__ActionPlay_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      crb_ros_msg__action__ActionPlay_Result__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

crb_ros_msg__action__ActionPlay_Result__Sequence *
crb_ros_msg__action__ActionPlay_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_Result__Sequence * array = (crb_ros_msg__action__ActionPlay_Result__Sequence *)allocator.allocate(sizeof(crb_ros_msg__action__ActionPlay_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crb_ros_msg__action__ActionPlay_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crb_ros_msg__action__ActionPlay_Result__Sequence__destroy(crb_ros_msg__action__ActionPlay_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crb_ros_msg__action__ActionPlay_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crb_ros_msg__action__ActionPlay_Result__Sequence__are_equal(const crb_ros_msg__action__ActionPlay_Result__Sequence * lhs, const crb_ros_msg__action__ActionPlay_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crb_ros_msg__action__ActionPlay_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crb_ros_msg__action__ActionPlay_Result__Sequence__copy(
  const crb_ros_msg__action__ActionPlay_Result__Sequence * input,
  crb_ros_msg__action__ActionPlay_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crb_ros_msg__action__ActionPlay_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crb_ros_msg__action__ActionPlay_Result * data =
      (crb_ros_msg__action__ActionPlay_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crb_ros_msg__action__ActionPlay_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crb_ros_msg__action__ActionPlay_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crb_ros_msg__action__ActionPlay_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
crb_ros_msg__action__ActionPlay_Feedback__init(crb_ros_msg__action__ActionPlay_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // action_index
  // exec_time
  // state
  return true;
}

void
crb_ros_msg__action__ActionPlay_Feedback__fini(crb_ros_msg__action__ActionPlay_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // action_index
  // exec_time
  // state
}

bool
crb_ros_msg__action__ActionPlay_Feedback__are_equal(const crb_ros_msg__action__ActionPlay_Feedback * lhs, const crb_ros_msg__action__ActionPlay_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // action_index
  if (lhs->action_index != rhs->action_index) {
    return false;
  }
  // exec_time
  if (lhs->exec_time != rhs->exec_time) {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  return true;
}

bool
crb_ros_msg__action__ActionPlay_Feedback__copy(
  const crb_ros_msg__action__ActionPlay_Feedback * input,
  crb_ros_msg__action__ActionPlay_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // action_index
  output->action_index = input->action_index;
  // exec_time
  output->exec_time = input->exec_time;
  // state
  output->state = input->state;
  return true;
}

crb_ros_msg__action__ActionPlay_Feedback *
crb_ros_msg__action__ActionPlay_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_Feedback * msg = (crb_ros_msg__action__ActionPlay_Feedback *)allocator.allocate(sizeof(crb_ros_msg__action__ActionPlay_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crb_ros_msg__action__ActionPlay_Feedback));
  bool success = crb_ros_msg__action__ActionPlay_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crb_ros_msg__action__ActionPlay_Feedback__destroy(crb_ros_msg__action__ActionPlay_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crb_ros_msg__action__ActionPlay_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crb_ros_msg__action__ActionPlay_Feedback__Sequence__init(crb_ros_msg__action__ActionPlay_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_Feedback * data = NULL;

  if (size) {
    data = (crb_ros_msg__action__ActionPlay_Feedback *)allocator.zero_allocate(size, sizeof(crb_ros_msg__action__ActionPlay_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crb_ros_msg__action__ActionPlay_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crb_ros_msg__action__ActionPlay_Feedback__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
crb_ros_msg__action__ActionPlay_Feedback__Sequence__fini(crb_ros_msg__action__ActionPlay_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      crb_ros_msg__action__ActionPlay_Feedback__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

crb_ros_msg__action__ActionPlay_Feedback__Sequence *
crb_ros_msg__action__ActionPlay_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_Feedback__Sequence * array = (crb_ros_msg__action__ActionPlay_Feedback__Sequence *)allocator.allocate(sizeof(crb_ros_msg__action__ActionPlay_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crb_ros_msg__action__ActionPlay_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crb_ros_msg__action__ActionPlay_Feedback__Sequence__destroy(crb_ros_msg__action__ActionPlay_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crb_ros_msg__action__ActionPlay_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crb_ros_msg__action__ActionPlay_Feedback__Sequence__are_equal(const crb_ros_msg__action__ActionPlay_Feedback__Sequence * lhs, const crb_ros_msg__action__ActionPlay_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crb_ros_msg__action__ActionPlay_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crb_ros_msg__action__ActionPlay_Feedback__Sequence__copy(
  const crb_ros_msg__action__ActionPlay_Feedback__Sequence * input,
  crb_ros_msg__action__ActionPlay_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crb_ros_msg__action__ActionPlay_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crb_ros_msg__action__ActionPlay_Feedback * data =
      (crb_ros_msg__action__ActionPlay_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crb_ros_msg__action__ActionPlay_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crb_ros_msg__action__ActionPlay_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crb_ros_msg__action__ActionPlay_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "crb_ros_msg/action/detail/action_play__functions.h"

bool
crb_ros_msg__action__ActionPlay_SendGoal_Request__init(crb_ros_msg__action__ActionPlay_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    crb_ros_msg__action__ActionPlay_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!crb_ros_msg__action__ActionPlay_Goal__init(&msg->goal)) {
    crb_ros_msg__action__ActionPlay_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
crb_ros_msg__action__ActionPlay_SendGoal_Request__fini(crb_ros_msg__action__ActionPlay_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  crb_ros_msg__action__ActionPlay_Goal__fini(&msg->goal);
}

bool
crb_ros_msg__action__ActionPlay_SendGoal_Request__are_equal(const crb_ros_msg__action__ActionPlay_SendGoal_Request * lhs, const crb_ros_msg__action__ActionPlay_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!crb_ros_msg__action__ActionPlay_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
crb_ros_msg__action__ActionPlay_SendGoal_Request__copy(
  const crb_ros_msg__action__ActionPlay_SendGoal_Request * input,
  crb_ros_msg__action__ActionPlay_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!crb_ros_msg__action__ActionPlay_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

crb_ros_msg__action__ActionPlay_SendGoal_Request *
crb_ros_msg__action__ActionPlay_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_SendGoal_Request * msg = (crb_ros_msg__action__ActionPlay_SendGoal_Request *)allocator.allocate(sizeof(crb_ros_msg__action__ActionPlay_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crb_ros_msg__action__ActionPlay_SendGoal_Request));
  bool success = crb_ros_msg__action__ActionPlay_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crb_ros_msg__action__ActionPlay_SendGoal_Request__destroy(crb_ros_msg__action__ActionPlay_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crb_ros_msg__action__ActionPlay_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence__init(crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_SendGoal_Request * data = NULL;

  if (size) {
    data = (crb_ros_msg__action__ActionPlay_SendGoal_Request *)allocator.zero_allocate(size, sizeof(crb_ros_msg__action__ActionPlay_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crb_ros_msg__action__ActionPlay_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crb_ros_msg__action__ActionPlay_SendGoal_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence__fini(crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      crb_ros_msg__action__ActionPlay_SendGoal_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence *
crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence * array = (crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence *)allocator.allocate(sizeof(crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence__destroy(crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence__are_equal(const crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence * lhs, const crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crb_ros_msg__action__ActionPlay_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence__copy(
  const crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence * input,
  crb_ros_msg__action__ActionPlay_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crb_ros_msg__action__ActionPlay_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crb_ros_msg__action__ActionPlay_SendGoal_Request * data =
      (crb_ros_msg__action__ActionPlay_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crb_ros_msg__action__ActionPlay_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crb_ros_msg__action__ActionPlay_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crb_ros_msg__action__ActionPlay_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
crb_ros_msg__action__ActionPlay_SendGoal_Response__init(crb_ros_msg__action__ActionPlay_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    crb_ros_msg__action__ActionPlay_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
crb_ros_msg__action__ActionPlay_SendGoal_Response__fini(crb_ros_msg__action__ActionPlay_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
crb_ros_msg__action__ActionPlay_SendGoal_Response__are_equal(const crb_ros_msg__action__ActionPlay_SendGoal_Response * lhs, const crb_ros_msg__action__ActionPlay_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
crb_ros_msg__action__ActionPlay_SendGoal_Response__copy(
  const crb_ros_msg__action__ActionPlay_SendGoal_Response * input,
  crb_ros_msg__action__ActionPlay_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

crb_ros_msg__action__ActionPlay_SendGoal_Response *
crb_ros_msg__action__ActionPlay_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_SendGoal_Response * msg = (crb_ros_msg__action__ActionPlay_SendGoal_Response *)allocator.allocate(sizeof(crb_ros_msg__action__ActionPlay_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crb_ros_msg__action__ActionPlay_SendGoal_Response));
  bool success = crb_ros_msg__action__ActionPlay_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crb_ros_msg__action__ActionPlay_SendGoal_Response__destroy(crb_ros_msg__action__ActionPlay_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crb_ros_msg__action__ActionPlay_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence__init(crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_SendGoal_Response * data = NULL;

  if (size) {
    data = (crb_ros_msg__action__ActionPlay_SendGoal_Response *)allocator.zero_allocate(size, sizeof(crb_ros_msg__action__ActionPlay_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crb_ros_msg__action__ActionPlay_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crb_ros_msg__action__ActionPlay_SendGoal_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence__fini(crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      crb_ros_msg__action__ActionPlay_SendGoal_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence *
crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence * array = (crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence *)allocator.allocate(sizeof(crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence__destroy(crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence__are_equal(const crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence * lhs, const crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crb_ros_msg__action__ActionPlay_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence__copy(
  const crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence * input,
  crb_ros_msg__action__ActionPlay_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crb_ros_msg__action__ActionPlay_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crb_ros_msg__action__ActionPlay_SendGoal_Response * data =
      (crb_ros_msg__action__ActionPlay_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crb_ros_msg__action__ActionPlay_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crb_ros_msg__action__ActionPlay_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crb_ros_msg__action__ActionPlay_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
crb_ros_msg__action__ActionPlay_GetResult_Request__init(crb_ros_msg__action__ActionPlay_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    crb_ros_msg__action__ActionPlay_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
crb_ros_msg__action__ActionPlay_GetResult_Request__fini(crb_ros_msg__action__ActionPlay_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
crb_ros_msg__action__ActionPlay_GetResult_Request__are_equal(const crb_ros_msg__action__ActionPlay_GetResult_Request * lhs, const crb_ros_msg__action__ActionPlay_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
crb_ros_msg__action__ActionPlay_GetResult_Request__copy(
  const crb_ros_msg__action__ActionPlay_GetResult_Request * input,
  crb_ros_msg__action__ActionPlay_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

crb_ros_msg__action__ActionPlay_GetResult_Request *
crb_ros_msg__action__ActionPlay_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_GetResult_Request * msg = (crb_ros_msg__action__ActionPlay_GetResult_Request *)allocator.allocate(sizeof(crb_ros_msg__action__ActionPlay_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crb_ros_msg__action__ActionPlay_GetResult_Request));
  bool success = crb_ros_msg__action__ActionPlay_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crb_ros_msg__action__ActionPlay_GetResult_Request__destroy(crb_ros_msg__action__ActionPlay_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crb_ros_msg__action__ActionPlay_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence__init(crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_GetResult_Request * data = NULL;

  if (size) {
    data = (crb_ros_msg__action__ActionPlay_GetResult_Request *)allocator.zero_allocate(size, sizeof(crb_ros_msg__action__ActionPlay_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crb_ros_msg__action__ActionPlay_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crb_ros_msg__action__ActionPlay_GetResult_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence__fini(crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      crb_ros_msg__action__ActionPlay_GetResult_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence *
crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence * array = (crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence *)allocator.allocate(sizeof(crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence__destroy(crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence__are_equal(const crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence * lhs, const crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crb_ros_msg__action__ActionPlay_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence__copy(
  const crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence * input,
  crb_ros_msg__action__ActionPlay_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crb_ros_msg__action__ActionPlay_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crb_ros_msg__action__ActionPlay_GetResult_Request * data =
      (crb_ros_msg__action__ActionPlay_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crb_ros_msg__action__ActionPlay_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crb_ros_msg__action__ActionPlay_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crb_ros_msg__action__ActionPlay_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "crb_ros_msg/action/detail/action_play__functions.h"

bool
crb_ros_msg__action__ActionPlay_GetResult_Response__init(crb_ros_msg__action__ActionPlay_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!crb_ros_msg__action__ActionPlay_Result__init(&msg->result)) {
    crb_ros_msg__action__ActionPlay_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
crb_ros_msg__action__ActionPlay_GetResult_Response__fini(crb_ros_msg__action__ActionPlay_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  crb_ros_msg__action__ActionPlay_Result__fini(&msg->result);
}

bool
crb_ros_msg__action__ActionPlay_GetResult_Response__are_equal(const crb_ros_msg__action__ActionPlay_GetResult_Response * lhs, const crb_ros_msg__action__ActionPlay_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!crb_ros_msg__action__ActionPlay_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
crb_ros_msg__action__ActionPlay_GetResult_Response__copy(
  const crb_ros_msg__action__ActionPlay_GetResult_Response * input,
  crb_ros_msg__action__ActionPlay_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!crb_ros_msg__action__ActionPlay_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

crb_ros_msg__action__ActionPlay_GetResult_Response *
crb_ros_msg__action__ActionPlay_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_GetResult_Response * msg = (crb_ros_msg__action__ActionPlay_GetResult_Response *)allocator.allocate(sizeof(crb_ros_msg__action__ActionPlay_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crb_ros_msg__action__ActionPlay_GetResult_Response));
  bool success = crb_ros_msg__action__ActionPlay_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crb_ros_msg__action__ActionPlay_GetResult_Response__destroy(crb_ros_msg__action__ActionPlay_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crb_ros_msg__action__ActionPlay_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence__init(crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_GetResult_Response * data = NULL;

  if (size) {
    data = (crb_ros_msg__action__ActionPlay_GetResult_Response *)allocator.zero_allocate(size, sizeof(crb_ros_msg__action__ActionPlay_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crb_ros_msg__action__ActionPlay_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crb_ros_msg__action__ActionPlay_GetResult_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence__fini(crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      crb_ros_msg__action__ActionPlay_GetResult_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence *
crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence * array = (crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence *)allocator.allocate(sizeof(crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence__destroy(crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence__are_equal(const crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence * lhs, const crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crb_ros_msg__action__ActionPlay_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence__copy(
  const crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence * input,
  crb_ros_msg__action__ActionPlay_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crb_ros_msg__action__ActionPlay_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crb_ros_msg__action__ActionPlay_GetResult_Response * data =
      (crb_ros_msg__action__ActionPlay_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crb_ros_msg__action__ActionPlay_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crb_ros_msg__action__ActionPlay_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crb_ros_msg__action__ActionPlay_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "crb_ros_msg/action/detail/action_play__functions.h"

bool
crb_ros_msg__action__ActionPlay_FeedbackMessage__init(crb_ros_msg__action__ActionPlay_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    crb_ros_msg__action__ActionPlay_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!crb_ros_msg__action__ActionPlay_Feedback__init(&msg->feedback)) {
    crb_ros_msg__action__ActionPlay_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
crb_ros_msg__action__ActionPlay_FeedbackMessage__fini(crb_ros_msg__action__ActionPlay_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  crb_ros_msg__action__ActionPlay_Feedback__fini(&msg->feedback);
}

bool
crb_ros_msg__action__ActionPlay_FeedbackMessage__are_equal(const crb_ros_msg__action__ActionPlay_FeedbackMessage * lhs, const crb_ros_msg__action__ActionPlay_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!crb_ros_msg__action__ActionPlay_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
crb_ros_msg__action__ActionPlay_FeedbackMessage__copy(
  const crb_ros_msg__action__ActionPlay_FeedbackMessage * input,
  crb_ros_msg__action__ActionPlay_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!crb_ros_msg__action__ActionPlay_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

crb_ros_msg__action__ActionPlay_FeedbackMessage *
crb_ros_msg__action__ActionPlay_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_FeedbackMessage * msg = (crb_ros_msg__action__ActionPlay_FeedbackMessage *)allocator.allocate(sizeof(crb_ros_msg__action__ActionPlay_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crb_ros_msg__action__ActionPlay_FeedbackMessage));
  bool success = crb_ros_msg__action__ActionPlay_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crb_ros_msg__action__ActionPlay_FeedbackMessage__destroy(crb_ros_msg__action__ActionPlay_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crb_ros_msg__action__ActionPlay_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence__init(crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_FeedbackMessage * data = NULL;

  if (size) {
    data = (crb_ros_msg__action__ActionPlay_FeedbackMessage *)allocator.zero_allocate(size, sizeof(crb_ros_msg__action__ActionPlay_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crb_ros_msg__action__ActionPlay_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crb_ros_msg__action__ActionPlay_FeedbackMessage__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence__fini(crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      crb_ros_msg__action__ActionPlay_FeedbackMessage__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence *
crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence * array = (crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence *)allocator.allocate(sizeof(crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence__destroy(crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence__are_equal(const crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence * lhs, const crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crb_ros_msg__action__ActionPlay_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence__copy(
  const crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence * input,
  crb_ros_msg__action__ActionPlay_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crb_ros_msg__action__ActionPlay_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crb_ros_msg__action__ActionPlay_FeedbackMessage * data =
      (crb_ros_msg__action__ActionPlay_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crb_ros_msg__action__ActionPlay_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crb_ros_msg__action__ActionPlay_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crb_ros_msg__action__ActionPlay_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
