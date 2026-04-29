#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "crb_ros_msg__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__crb_ros_msg__msg__UpperJointData() -> *const std::ffi::c_void;
}

#[link(name = "crb_ros_msg__rosidl_generator_c")]
extern "C" {
    fn crb_ros_msg__msg__UpperJointData__init(msg: *mut UpperJointData) -> bool;
    fn crb_ros_msg__msg__UpperJointData__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<UpperJointData>, size: usize) -> bool;
    fn crb_ros_msg__msg__UpperJointData__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<UpperJointData>);
    fn crb_ros_msg__msg__UpperJointData__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<UpperJointData>, out_seq: *mut rosidl_runtime_rs::Sequence<UpperJointData>) -> bool;
}

// Corresponds to crb_ros_msg__msg__UpperJointData
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UpperJointData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub time_ref: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vel_scale: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint: sensor_msgs::msg::rmw::JointState,

}



impl Default for UpperJointData {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !crb_ros_msg__msg__UpperJointData__init(&mut msg as *mut _) {
        panic!("Call to crb_ros_msg__msg__UpperJointData__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for UpperJointData {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { crb_ros_msg__msg__UpperJointData__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { crb_ros_msg__msg__UpperJointData__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { crb_ros_msg__msg__UpperJointData__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for UpperJointData {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for UpperJointData where Self: Sized {
  const TYPE_NAME: &'static str = "crb_ros_msg/msg/UpperJointData";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__crb_ros_msg__msg__UpperJointData() }
  }
}


