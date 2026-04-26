// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crb_ros_msg:msg/UpperJointData.idl
// generated code does not contain a copyright notice

#ifndef CRB_ROS_MSG__MSG__DETAIL__UPPER_JOINT_DATA__TRAITS_HPP_
#define CRB_ROS_MSG__MSG__DETAIL__UPPER_JOINT_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crb_ros_msg/msg/detail/upper_joint_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'joint'
#include "sensor_msgs/msg/detail/joint_state__traits.hpp"

namespace crb_ros_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const UpperJointData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: time_ref
  {
    out << "time_ref: ";
    rosidl_generator_traits::value_to_yaml(msg.time_ref, out);
    out << ", ";
  }

  // member: vel_scale
  {
    out << "vel_scale: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_scale, out);
    out << ", ";
  }

  // member: joint
  {
    out << "joint: ";
    to_flow_style_yaml(msg.joint, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UpperJointData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: time_ref
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_ref: ";
    rosidl_generator_traits::value_to_yaml(msg.time_ref, out);
    out << "\n";
  }

  // member: vel_scale
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_scale: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_scale, out);
    out << "\n";
  }

  // member: joint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint:\n";
    to_block_style_yaml(msg.joint, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UpperJointData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace crb_ros_msg

namespace rosidl_generator_traits
{

[[deprecated("use crb_ros_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const crb_ros_msg::msg::UpperJointData & msg,
  std::ostream & out, size_t indentation = 0)
{
  crb_ros_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crb_ros_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const crb_ros_msg::msg::UpperJointData & msg)
{
  return crb_ros_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<crb_ros_msg::msg::UpperJointData>()
{
  return "crb_ros_msg::msg::UpperJointData";
}

template<>
inline const char * name<crb_ros_msg::msg::UpperJointData>()
{
  return "crb_ros_msg/msg/UpperJointData";
}

template<>
struct has_fixed_size<crb_ros_msg::msg::UpperJointData>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::JointState>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<crb_ros_msg::msg::UpperJointData>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::JointState>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<crb_ros_msg::msg::UpperJointData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CRB_ROS_MSG__MSG__DETAIL__UPPER_JOINT_DATA__TRAITS_HPP_
