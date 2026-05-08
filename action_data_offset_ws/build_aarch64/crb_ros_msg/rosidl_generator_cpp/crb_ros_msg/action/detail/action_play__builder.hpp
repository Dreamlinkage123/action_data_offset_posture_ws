// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crb_ros_msg:action/ActionPlay.idl
// generated code does not contain a copyright notice

#ifndef CRB_ROS_MSG__ACTION__DETAIL__ACTION_PLAY__BUILDER_HPP_
#define CRB_ROS_MSG__ACTION__DETAIL__ACTION_PLAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crb_ros_msg/action/detail/action_play__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crb_ros_msg
{

namespace action
{

namespace builder
{

class Init_ActionPlay_Goal_rl_name
{
public:
  explicit Init_ActionPlay_Goal_rl_name(::crb_ros_msg::action::ActionPlay_Goal & msg)
  : msg_(msg)
  {}
  ::crb_ros_msg::action::ActionPlay_Goal rl_name(::crb_ros_msg::action::ActionPlay_Goal::_rl_name_type arg)
  {
    msg_.rl_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crb_ros_msg::action::ActionPlay_Goal msg_;
};

class Init_ActionPlay_Goal_cancel_action_name
{
public:
  explicit Init_ActionPlay_Goal_cancel_action_name(::crb_ros_msg::action::ActionPlay_Goal & msg)
  : msg_(msg)
  {}
  Init_ActionPlay_Goal_rl_name cancel_action_name(::crb_ros_msg::action::ActionPlay_Goal::_cancel_action_name_type arg)
  {
    msg_.cancel_action_name = std::move(arg);
    return Init_ActionPlay_Goal_rl_name(msg_);
  }

private:
  ::crb_ros_msg::action::ActionPlay_Goal msg_;
};

class Init_ActionPlay_Goal_action_name
{
public:
  explicit Init_ActionPlay_Goal_action_name(::crb_ros_msg::action::ActionPlay_Goal & msg)
  : msg_(msg)
  {}
  Init_ActionPlay_Goal_cancel_action_name action_name(::crb_ros_msg::action::ActionPlay_Goal::_action_name_type arg)
  {
    msg_.action_name = std::move(arg);
    return Init_ActionPlay_Goal_cancel_action_name(msg_);
  }

private:
  ::crb_ros_msg::action::ActionPlay_Goal msg_;
};

class Init_ActionPlay_Goal_start_time
{
public:
  Init_ActionPlay_Goal_start_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ActionPlay_Goal_action_name start_time(::crb_ros_msg::action::ActionPlay_Goal::_start_time_type arg)
  {
    msg_.start_time = std::move(arg);
    return Init_ActionPlay_Goal_action_name(msg_);
  }

private:
  ::crb_ros_msg::action::ActionPlay_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::crb_ros_msg::action::ActionPlay_Goal>()
{
  return crb_ros_msg::action::builder::Init_ActionPlay_Goal_start_time();
}

}  // namespace crb_ros_msg


namespace crb_ros_msg
{

namespace action
{

namespace builder
{

class Init_ActionPlay_Result_if_success
{
public:
  Init_ActionPlay_Result_if_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::crb_ros_msg::action::ActionPlay_Result if_success(::crb_ros_msg::action::ActionPlay_Result::_if_success_type arg)
  {
    msg_.if_success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crb_ros_msg::action::ActionPlay_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::crb_ros_msg::action::ActionPlay_Result>()
{
  return crb_ros_msg::action::builder::Init_ActionPlay_Result_if_success();
}

}  // namespace crb_ros_msg


namespace crb_ros_msg
{

namespace action
{

namespace builder
{

class Init_ActionPlay_Feedback_state
{
public:
  explicit Init_ActionPlay_Feedback_state(::crb_ros_msg::action::ActionPlay_Feedback & msg)
  : msg_(msg)
  {}
  ::crb_ros_msg::action::ActionPlay_Feedback state(::crb_ros_msg::action::ActionPlay_Feedback::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crb_ros_msg::action::ActionPlay_Feedback msg_;
};

class Init_ActionPlay_Feedback_exec_time
{
public:
  explicit Init_ActionPlay_Feedback_exec_time(::crb_ros_msg::action::ActionPlay_Feedback & msg)
  : msg_(msg)
  {}
  Init_ActionPlay_Feedback_state exec_time(::crb_ros_msg::action::ActionPlay_Feedback::_exec_time_type arg)
  {
    msg_.exec_time = std::move(arg);
    return Init_ActionPlay_Feedback_state(msg_);
  }

private:
  ::crb_ros_msg::action::ActionPlay_Feedback msg_;
};

class Init_ActionPlay_Feedback_action_index
{
public:
  Init_ActionPlay_Feedback_action_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ActionPlay_Feedback_exec_time action_index(::crb_ros_msg::action::ActionPlay_Feedback::_action_index_type arg)
  {
    msg_.action_index = std::move(arg);
    return Init_ActionPlay_Feedback_exec_time(msg_);
  }

private:
  ::crb_ros_msg::action::ActionPlay_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::crb_ros_msg::action::ActionPlay_Feedback>()
{
  return crb_ros_msg::action::builder::Init_ActionPlay_Feedback_action_index();
}

}  // namespace crb_ros_msg


namespace crb_ros_msg
{

namespace action
{

namespace builder
{

class Init_ActionPlay_SendGoal_Request_goal
{
public:
  explicit Init_ActionPlay_SendGoal_Request_goal(::crb_ros_msg::action::ActionPlay_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::crb_ros_msg::action::ActionPlay_SendGoal_Request goal(::crb_ros_msg::action::ActionPlay_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crb_ros_msg::action::ActionPlay_SendGoal_Request msg_;
};

class Init_ActionPlay_SendGoal_Request_goal_id
{
public:
  Init_ActionPlay_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ActionPlay_SendGoal_Request_goal goal_id(::crb_ros_msg::action::ActionPlay_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ActionPlay_SendGoal_Request_goal(msg_);
  }

private:
  ::crb_ros_msg::action::ActionPlay_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::crb_ros_msg::action::ActionPlay_SendGoal_Request>()
{
  return crb_ros_msg::action::builder::Init_ActionPlay_SendGoal_Request_goal_id();
}

}  // namespace crb_ros_msg


namespace crb_ros_msg
{

namespace action
{

namespace builder
{

class Init_ActionPlay_SendGoal_Response_stamp
{
public:
  explicit Init_ActionPlay_SendGoal_Response_stamp(::crb_ros_msg::action::ActionPlay_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::crb_ros_msg::action::ActionPlay_SendGoal_Response stamp(::crb_ros_msg::action::ActionPlay_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crb_ros_msg::action::ActionPlay_SendGoal_Response msg_;
};

class Init_ActionPlay_SendGoal_Response_accepted
{
public:
  Init_ActionPlay_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ActionPlay_SendGoal_Response_stamp accepted(::crb_ros_msg::action::ActionPlay_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_ActionPlay_SendGoal_Response_stamp(msg_);
  }

private:
  ::crb_ros_msg::action::ActionPlay_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::crb_ros_msg::action::ActionPlay_SendGoal_Response>()
{
  return crb_ros_msg::action::builder::Init_ActionPlay_SendGoal_Response_accepted();
}

}  // namespace crb_ros_msg


namespace crb_ros_msg
{

namespace action
{

namespace builder
{

class Init_ActionPlay_GetResult_Request_goal_id
{
public:
  Init_ActionPlay_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::crb_ros_msg::action::ActionPlay_GetResult_Request goal_id(::crb_ros_msg::action::ActionPlay_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crb_ros_msg::action::ActionPlay_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::crb_ros_msg::action::ActionPlay_GetResult_Request>()
{
  return crb_ros_msg::action::builder::Init_ActionPlay_GetResult_Request_goal_id();
}

}  // namespace crb_ros_msg


namespace crb_ros_msg
{

namespace action
{

namespace builder
{

class Init_ActionPlay_GetResult_Response_result
{
public:
  explicit Init_ActionPlay_GetResult_Response_result(::crb_ros_msg::action::ActionPlay_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::crb_ros_msg::action::ActionPlay_GetResult_Response result(::crb_ros_msg::action::ActionPlay_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crb_ros_msg::action::ActionPlay_GetResult_Response msg_;
};

class Init_ActionPlay_GetResult_Response_status
{
public:
  Init_ActionPlay_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ActionPlay_GetResult_Response_result status(::crb_ros_msg::action::ActionPlay_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ActionPlay_GetResult_Response_result(msg_);
  }

private:
  ::crb_ros_msg::action::ActionPlay_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::crb_ros_msg::action::ActionPlay_GetResult_Response>()
{
  return crb_ros_msg::action::builder::Init_ActionPlay_GetResult_Response_status();
}

}  // namespace crb_ros_msg


namespace crb_ros_msg
{

namespace action
{

namespace builder
{

class Init_ActionPlay_FeedbackMessage_feedback
{
public:
  explicit Init_ActionPlay_FeedbackMessage_feedback(::crb_ros_msg::action::ActionPlay_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::crb_ros_msg::action::ActionPlay_FeedbackMessage feedback(::crb_ros_msg::action::ActionPlay_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crb_ros_msg::action::ActionPlay_FeedbackMessage msg_;
};

class Init_ActionPlay_FeedbackMessage_goal_id
{
public:
  Init_ActionPlay_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ActionPlay_FeedbackMessage_feedback goal_id(::crb_ros_msg::action::ActionPlay_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ActionPlay_FeedbackMessage_feedback(msg_);
  }

private:
  ::crb_ros_msg::action::ActionPlay_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::crb_ros_msg::action::ActionPlay_FeedbackMessage>()
{
  return crb_ros_msg::action::builder::Init_ActionPlay_FeedbackMessage_goal_id();
}

}  // namespace crb_ros_msg

#endif  // CRB_ROS_MSG__ACTION__DETAIL__ACTION_PLAY__BUILDER_HPP_
