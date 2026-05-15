
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to crb_ros_msg__action__ActionPlay_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ActionPlay_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub start_time: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub action_name: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cancel_action_name: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rl_name: std::string::String,

}



impl Default for ActionPlay_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ActionPlay_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for ActionPlay_Goal {
  type RmwMsg = super::action::rmw::ActionPlay_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        start_time: msg.start_time,
        action_name: msg.action_name.as_str().into(),
        cancel_action_name: msg.cancel_action_name.as_str().into(),
        rl_name: msg.rl_name.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      start_time: msg.start_time,
        action_name: msg.action_name.as_str().into(),
        cancel_action_name: msg.cancel_action_name.as_str().into(),
        rl_name: msg.rl_name.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      start_time: msg.start_time,
      action_name: msg.action_name.to_string(),
      cancel_action_name: msg.cancel_action_name.to_string(),
      rl_name: msg.rl_name.to_string(),
    }
  }
}


// Corresponds to crb_ros_msg__action__ActionPlay_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ActionPlay_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub if_success: bool,

}



impl Default for ActionPlay_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ActionPlay_Result::default())
  }
}

impl rosidl_runtime_rs::Message for ActionPlay_Result {
  type RmwMsg = super::action::rmw::ActionPlay_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        if_success: msg.if_success,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      if_success: msg.if_success,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      if_success: msg.if_success,
    }
  }
}


// Corresponds to crb_ros_msg__action__ActionPlay_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ActionPlay_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub action_index: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub exec_time: f64,

    /// 状态，0：未播放，1：加载文件中；2：动作播放中；3：动作结束; 4: 动作取消； 5： 动作取消结束
    pub state: i32,

}



impl Default for ActionPlay_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ActionPlay_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for ActionPlay_Feedback {
  type RmwMsg = super::action::rmw::ActionPlay_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        action_index: msg.action_index,
        exec_time: msg.exec_time,
        state: msg.state,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      action_index: msg.action_index,
      exec_time: msg.exec_time,
      state: msg.state,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      action_index: msg.action_index,
      exec_time: msg.exec_time,
      state: msg.state,
    }
  }
}


// Corresponds to crb_ros_msg__action__ActionPlay_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ActionPlay_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::ActionPlay_Feedback,

}



impl Default for ActionPlay_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ActionPlay_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for ActionPlay_FeedbackMessage {
  type RmwMsg = super::action::rmw::ActionPlay_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::ActionPlay_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::ActionPlay_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::ActionPlay_Feedback::from_rmw_message(msg.feedback),
    }
  }
}






// Corresponds to crb_ros_msg__action__ActionPlay_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ActionPlay_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::ActionPlay_Goal,

}



impl Default for ActionPlay_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ActionPlay_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ActionPlay_SendGoal_Request {
  type RmwMsg = super::action::rmw::ActionPlay_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::ActionPlay_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::ActionPlay_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::ActionPlay_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to crb_ros_msg__action__ActionPlay_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ActionPlay_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for ActionPlay_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ActionPlay_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ActionPlay_SendGoal_Response {
  type RmwMsg = super::action::rmw::ActionPlay_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to crb_ros_msg__action__ActionPlay_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ActionPlay_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for ActionPlay_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ActionPlay_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ActionPlay_GetResult_Request {
  type RmwMsg = super::action::rmw::ActionPlay_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to crb_ros_msg__action__ActionPlay_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ActionPlay_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::ActionPlay_Result,

}



impl Default for ActionPlay_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ActionPlay_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ActionPlay_GetResult_Response {
  type RmwMsg = super::action::rmw::ActionPlay_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::ActionPlay_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::ActionPlay_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::ActionPlay_Result::from_rmw_message(msg.result),
    }
  }
}






#[link(name = "crb_ros_msg__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__crb_ros_msg__action__ActionPlay_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to crb_ros_msg__action__ActionPlay_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct ActionPlay_SendGoal;

impl rosidl_runtime_rs::Service for ActionPlay_SendGoal {
    type Request = ActionPlay_SendGoal_Request;
    type Response = ActionPlay_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__crb_ros_msg__action__ActionPlay_SendGoal() }
    }
}




#[link(name = "crb_ros_msg__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__crb_ros_msg__action__ActionPlay_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to crb_ros_msg__action__ActionPlay_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct ActionPlay_GetResult;

impl rosidl_runtime_rs::Service for ActionPlay_GetResult {
    type Request = ActionPlay_GetResult_Request;
    type Response = ActionPlay_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__crb_ros_msg__action__ActionPlay_GetResult() }
    }
}






#[link(name = "crb_ros_msg__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__crb_ros_msg__action__ActionPlay() -> *const std::ffi::c_void;
}

// Corresponds to crb_ros_msg__action__ActionPlay
#[allow(missing_docs, non_camel_case_types)]
pub struct ActionPlay;

impl rosidl_runtime_rs::Action for ActionPlay {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = ActionPlay_Goal;

  /// The result message defined in the action definition.
  type Result = ActionPlay_Result;

  /// The feedback message defined in the action definition.
  type Feedback = ActionPlay_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::ActionPlay_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::ActionPlay_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::ActionPlay_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__crb_ros_msg__action__ActionPlay() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::ActionPlay_Goal,
  ) -> super::action::rmw::ActionPlay_SendGoal_Request {
   super::action::rmw::ActionPlay_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::ActionPlay_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::ActionPlay_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::ActionPlay_SendGoal_Response {
   super::action::rmw::ActionPlay_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::ActionPlay_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::ActionPlay_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::ActionPlay_Feedback,
  ) -> super::action::rmw::ActionPlay_FeedbackMessage {
    let mut message = super::action::rmw::ActionPlay_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::ActionPlay_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::ActionPlay_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::ActionPlay_GetResult_Request {
   super::action::rmw::ActionPlay_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::ActionPlay_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::ActionPlay_Result,
  ) -> super::action::rmw::ActionPlay_GetResult_Response {
   super::action::rmw::ActionPlay_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::ActionPlay_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::ActionPlay_Result,
  ) {
    (response.status, response.result)
  }
}


