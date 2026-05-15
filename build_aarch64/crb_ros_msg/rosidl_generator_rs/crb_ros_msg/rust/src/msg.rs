#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to crb_ros_msg__msg__UpperJointData

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UpperJointData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub time_ref: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vel_scale: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint: sensor_msgs::msg::JointState,

}



impl Default for UpperJointData {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::UpperJointData::default())
  }
}

impl rosidl_runtime_rs::Message for UpperJointData {
  type RmwMsg = super::msg::rmw::UpperJointData;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        time_ref: msg.time_ref,
        vel_scale: msg.vel_scale,
        joint: sensor_msgs::msg::JointState::into_rmw_message(std::borrow::Cow::Owned(msg.joint)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      time_ref: msg.time_ref,
      vel_scale: msg.vel_scale,
        joint: sensor_msgs::msg::JointState::into_rmw_message(std::borrow::Cow::Borrowed(&msg.joint)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      time_ref: msg.time_ref,
      vel_scale: msg.vel_scale,
      joint: sensor_msgs::msg::JointState::from_rmw_message(msg.joint),
    }
  }
}


