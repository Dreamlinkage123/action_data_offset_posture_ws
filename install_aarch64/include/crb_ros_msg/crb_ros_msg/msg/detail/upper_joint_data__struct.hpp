// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from crb_ros_msg:msg/UpperJointData.idl
// generated code does not contain a copyright notice

#ifndef CRB_ROS_MSG__MSG__DETAIL__UPPER_JOINT_DATA__STRUCT_HPP_
#define CRB_ROS_MSG__MSG__DETAIL__UPPER_JOINT_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'joint'
#include "sensor_msgs/msg/detail/joint_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__crb_ros_msg__msg__UpperJointData __attribute__((deprecated))
#else
# define DEPRECATED__crb_ros_msg__msg__UpperJointData __declspec(deprecated)
#endif

namespace crb_ros_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UpperJointData_
{
  using Type = UpperJointData_<ContainerAllocator>;

  explicit UpperJointData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    joint(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_ref = 0.0f;
      this->vel_scale = 0.0f;
    }
  }

  explicit UpperJointData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    joint(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_ref = 0.0f;
      this->vel_scale = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _time_ref_type =
    float;
  _time_ref_type time_ref;
  using _vel_scale_type =
    float;
  _vel_scale_type vel_scale;
  using _joint_type =
    sensor_msgs::msg::JointState_<ContainerAllocator>;
  _joint_type joint;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__time_ref(
    const float & _arg)
  {
    this->time_ref = _arg;
    return *this;
  }
  Type & set__vel_scale(
    const float & _arg)
  {
    this->vel_scale = _arg;
    return *this;
  }
  Type & set__joint(
    const sensor_msgs::msg::JointState_<ContainerAllocator> & _arg)
  {
    this->joint = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crb_ros_msg::msg::UpperJointData_<ContainerAllocator> *;
  using ConstRawPtr =
    const crb_ros_msg::msg::UpperJointData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crb_ros_msg::msg::UpperJointData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crb_ros_msg::msg::UpperJointData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crb_ros_msg::msg::UpperJointData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crb_ros_msg::msg::UpperJointData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crb_ros_msg::msg::UpperJointData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crb_ros_msg::msg::UpperJointData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crb_ros_msg::msg::UpperJointData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crb_ros_msg::msg::UpperJointData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crb_ros_msg__msg__UpperJointData
    std::shared_ptr<crb_ros_msg::msg::UpperJointData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crb_ros_msg__msg__UpperJointData
    std::shared_ptr<crb_ros_msg::msg::UpperJointData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UpperJointData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->time_ref != other.time_ref) {
      return false;
    }
    if (this->vel_scale != other.vel_scale) {
      return false;
    }
    if (this->joint != other.joint) {
      return false;
    }
    return true;
  }
  bool operator!=(const UpperJointData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UpperJointData_

// alias to use template instance with default allocator
using UpperJointData =
  crb_ros_msg::msg::UpperJointData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace crb_ros_msg

#endif  // CRB_ROS_MSG__MSG__DETAIL__UPPER_JOINT_DATA__STRUCT_HPP_
