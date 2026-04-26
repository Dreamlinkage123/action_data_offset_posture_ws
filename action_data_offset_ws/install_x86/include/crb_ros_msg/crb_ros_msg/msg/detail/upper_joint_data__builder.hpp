// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crb_ros_msg:msg/UpperJointData.idl
// generated code does not contain a copyright notice

#ifndef CRB_ROS_MSG__MSG__DETAIL__UPPER_JOINT_DATA__BUILDER_HPP_
#define CRB_ROS_MSG__MSG__DETAIL__UPPER_JOINT_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crb_ros_msg/msg/detail/upper_joint_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crb_ros_msg
{

namespace msg
{

namespace builder
{

class Init_UpperJointData_joint
{
public:
  explicit Init_UpperJointData_joint(::crb_ros_msg::msg::UpperJointData & msg)
  : msg_(msg)
  {}
  ::crb_ros_msg::msg::UpperJointData joint(::crb_ros_msg::msg::UpperJointData::_joint_type arg)
  {
    msg_.joint = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crb_ros_msg::msg::UpperJointData msg_;
};

class Init_UpperJointData_vel_scale
{
public:
  explicit Init_UpperJointData_vel_scale(::crb_ros_msg::msg::UpperJointData & msg)
  : msg_(msg)
  {}
  Init_UpperJointData_joint vel_scale(::crb_ros_msg::msg::UpperJointData::_vel_scale_type arg)
  {
    msg_.vel_scale = std::move(arg);
    return Init_UpperJointData_joint(msg_);
  }

private:
  ::crb_ros_msg::msg::UpperJointData msg_;
};

class Init_UpperJointData_time_ref
{
public:
  explicit Init_UpperJointData_time_ref(::crb_ros_msg::msg::UpperJointData & msg)
  : msg_(msg)
  {}
  Init_UpperJointData_vel_scale time_ref(::crb_ros_msg::msg::UpperJointData::_time_ref_type arg)
  {
    msg_.time_ref = std::move(arg);
    return Init_UpperJointData_vel_scale(msg_);
  }

private:
  ::crb_ros_msg::msg::UpperJointData msg_;
};

class Init_UpperJointData_header
{
public:
  Init_UpperJointData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UpperJointData_time_ref header(::crb_ros_msg::msg::UpperJointData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UpperJointData_time_ref(msg_);
  }

private:
  ::crb_ros_msg::msg::UpperJointData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::crb_ros_msg::msg::UpperJointData>()
{
  return crb_ros_msg::msg::builder::Init_UpperJointData_header();
}

}  // namespace crb_ros_msg

#endif  // CRB_ROS_MSG__MSG__DETAIL__UPPER_JOINT_DATA__BUILDER_HPP_
