#include "omnibot_driver.h"

namespace omnibot_driver {

Omnibot::Omnibot()
    :
    front_right_cmd_(0),
    front_right_pos_(0),
    front_right_vel_(0),
    front_right_eff_(0),
    front_left_cmd_(0),
    front_left_pos_(0),
    front_left_vel_(0),
    front_left_eff_(0),
    rear_right_cmd_(0),
    rear_right_pos_(0),
    rear_right_vel_(0),
    rear_right_eff_(0),
    rear_left_cmd_(0),
    rear_left_pos_(0),
    rear_left_vel_(0),
    rear_left_eff_(0)
{
  ROS_ERROR("driver init");

  // Register front right wheel
  hardware_interface::JointStateHandle front_right_state_handle(
      "wheel_front_right_joint",
      &front_right_pos_,
      &front_right_vel_,
      &front_right_eff_
    );

  joint_state_interface_.registerHandle(front_right_state_handle);
  hardware_interface::JointHandle front_right_handle(
      joint_state_interface_.getHandle("wheel_front_right_joint"),
      &front_right_cmd_);
  joint_vel_interface_.registerHandle(front_right_handle);

  // Register front left wheel
  hardware_interface::JointStateHandle front_left_state_handle(
      "wheel_front_left_joint",
      &front_left_pos_,
      &front_left_vel_,
      &front_left_eff_
    );
  joint_state_interface_.registerHandle(front_left_state_handle);

  hardware_interface::JointHandle front_left_handle(
      joint_state_interface_.getHandle("wheel_front_left_joint"),
      &front_left_cmd_);
  joint_vel_interface_.registerHandle(front_left_handle);

  // Register rear right wheel
  hardware_interface::JointStateHandle rear_right_state_handle(
      "wheel_rear_right_joint",
      &rear_right_pos_,
      &rear_right_vel_,
      &rear_right_eff_
    );

  joint_state_interface_.registerHandle(rear_right_state_handle);
  hardware_interface::JointHandle rear_right_handle(
      joint_state_interface_.getHandle("wheel_rear_right_joint"),
      &rear_right_cmd_);
  joint_vel_interface_.registerHandle(rear_right_handle);

  // Register front left wheel
  hardware_interface::JointStateHandle rear_left_state_handle(
      "wheel_rear_left_joint",
      &rear_left_pos_,
      &rear_left_vel_,
      &rear_left_eff_
    );
  joint_state_interface_.registerHandle(rear_left_state_handle);

  hardware_interface::JointHandle rear_left_handle(
      joint_state_interface_.getHandle("wheel_rear_left_joint"),
      &rear_left_cmd_);
  joint_vel_interface_.registerHandle(rear_left_handle);

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&joint_vel_interface_);
}

void Omnibot::read() {
  ROS_ERROR("read");
}

void Omnibot::write() {
  ROS_ERROR("write");
  ROS_ERROR("Joint command front right: %f", front_right_cmd_);
  ROS_ERROR("Joint command front left: %f", front_left_cmd_);
  ROS_ERROR("Joint command rear right: %f", rear_right_cmd_);
  ROS_ERROR("Joint command rear left: %f", rear_left_cmd_);
}

}; // namespace omnibot_driver
