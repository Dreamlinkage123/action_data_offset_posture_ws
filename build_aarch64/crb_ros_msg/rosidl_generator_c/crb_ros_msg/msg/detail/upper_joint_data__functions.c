// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from crb_ros_msg:msg/UpperJointData.idl
// generated code does not contain a copyright notice
#include "crb_ros_msg/msg/detail/upper_joint_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `joint`
#include "sensor_msgs/msg/detail/joint_state__functions.h"

bool
crb_ros_msg__msg__UpperJointData__init(crb_ros_msg__msg__UpperJointData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    crb_ros_msg__msg__UpperJointData__fini(msg);
    return false;
  }
  // time_ref
  // vel_scale
  // joint
  if (!sensor_msgs__msg__JointState__init(&msg->joint)) {
    crb_ros_msg__msg__UpperJointData__fini(msg);
    return false;
  }
  return true;
}

void
crb_ros_msg__msg__UpperJointData__fini(crb_ros_msg__msg__UpperJointData * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // time_ref
  // vel_scale
  // joint
  sensor_msgs__msg__JointState__fini(&msg->joint);
}

bool
crb_ros_msg__msg__UpperJointData__are_equal(const crb_ros_msg__msg__UpperJointData * lhs, const crb_ros_msg__msg__UpperJointData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // time_ref
  if (lhs->time_ref != rhs->time_ref) {
    return false;
  }
  // vel_scale
  if (lhs->vel_scale != rhs->vel_scale) {
    return false;
  }
  // joint
  if (!sensor_msgs__msg__JointState__are_equal(
      &(lhs->joint), &(rhs->joint)))
  {
    return false;
  }
  return true;
}

bool
crb_ros_msg__msg__UpperJointData__copy(
  const crb_ros_msg__msg__UpperJointData * input,
  crb_ros_msg__msg__UpperJointData * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // time_ref
  output->time_ref = input->time_ref;
  // vel_scale
  output->vel_scale = input->vel_scale;
  // joint
  if (!sensor_msgs__msg__JointState__copy(
      &(input->joint), &(output->joint)))
  {
    return false;
  }
  return true;
}

crb_ros_msg__msg__UpperJointData *
crb_ros_msg__msg__UpperJointData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__msg__UpperJointData * msg = (crb_ros_msg__msg__UpperJointData *)allocator.allocate(sizeof(crb_ros_msg__msg__UpperJointData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crb_ros_msg__msg__UpperJointData));
  bool success = crb_ros_msg__msg__UpperJointData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crb_ros_msg__msg__UpperJointData__destroy(crb_ros_msg__msg__UpperJointData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crb_ros_msg__msg__UpperJointData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crb_ros_msg__msg__UpperJointData__Sequence__init(crb_ros_msg__msg__UpperJointData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__msg__UpperJointData * data = NULL;

  if (size) {
    data = (crb_ros_msg__msg__UpperJointData *)allocator.zero_allocate(size, sizeof(crb_ros_msg__msg__UpperJointData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crb_ros_msg__msg__UpperJointData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crb_ros_msg__msg__UpperJointData__fini(&data[i - 1]);
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
crb_ros_msg__msg__UpperJointData__Sequence__fini(crb_ros_msg__msg__UpperJointData__Sequence * array)
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
      crb_ros_msg__msg__UpperJointData__fini(&array->data[i]);
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

crb_ros_msg__msg__UpperJointData__Sequence *
crb_ros_msg__msg__UpperJointData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crb_ros_msg__msg__UpperJointData__Sequence * array = (crb_ros_msg__msg__UpperJointData__Sequence *)allocator.allocate(sizeof(crb_ros_msg__msg__UpperJointData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crb_ros_msg__msg__UpperJointData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crb_ros_msg__msg__UpperJointData__Sequence__destroy(crb_ros_msg__msg__UpperJointData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crb_ros_msg__msg__UpperJointData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crb_ros_msg__msg__UpperJointData__Sequence__are_equal(const crb_ros_msg__msg__UpperJointData__Sequence * lhs, const crb_ros_msg__msg__UpperJointData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crb_ros_msg__msg__UpperJointData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crb_ros_msg__msg__UpperJointData__Sequence__copy(
  const crb_ros_msg__msg__UpperJointData__Sequence * input,
  crb_ros_msg__msg__UpperJointData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crb_ros_msg__msg__UpperJointData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crb_ros_msg__msg__UpperJointData * data =
      (crb_ros_msg__msg__UpperJointData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crb_ros_msg__msg__UpperJointData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crb_ros_msg__msg__UpperJointData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crb_ros_msg__msg__UpperJointData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
