#include "omnibot_driver.h"
#include <serial/serial.h>
#include <vector>

namespace omnibot_driver {

  void enumerate_ports()
  {
  	std::vector<serial::PortInfo> devices_found = serial::list_ports();

  	std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

  	while( iter != devices_found.end() )
  	{
  		serial::PortInfo device = *iter++;

  		printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
       device.hardware_id.c_str() );
  	}
  }

Omnibot::Omnibot()
    :
    use_lowpass_(true),
    lowpass_constant_(0.02),
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
    rear_left_eff_(0),
    last_front_right_cmd_(0),
    last_front_left_cmd_(0),
    last_rear_right_cmd_(0),
    last_rear_left_cmd_(0),
    lowpass_front_right_cmd_(0),
    lowpass_front_left_cmd_(0),
    lowpass_rear_right_cmd_(0),
    lowpass_rear_left_cmd_(0)
{
  ROS_ERROR("driver init");

  registerJoints();
  registerJointLimits();

  registerInterface(&joint_state_interface_);
  registerInterface(&joint_vel_interface_);
}

void Omnibot::registerJoints() {
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

}

void Omnibot::registerJointLimits() {
  ros::NodeHandle nh;

  joint_limits_interface::JointLimits limits;
  joint_limits_interface::SoftJointLimits soft_limits;
  if(getJointLimits("mecanum_joints", nh, limits) == 0){
    ROS_ERROR("Joint limits not specified. Aborting!");
    throw;
}

  hardware_interface::JointHandle joint_handle;

  joint_handle  = joint_vel_interface_.getHandle("wheel_front_right_joint");
  joint_limits_interface::VelocityJointSoftLimitsHandle front_right_handle(
    joint_handle,
    limits,
    soft_limits
  );
  joint_limits_interface_.registerHandle(front_right_handle);

  joint_handle  = joint_vel_interface_.getHandle("wheel_front_left_joint");
  joint_limits_interface::VelocityJointSoftLimitsHandle front_left_handle(
    joint_handle,
    limits,
    soft_limits
  );
  joint_limits_interface_.registerHandle(front_left_handle);

  joint_handle  = joint_vel_interface_.getHandle("wheel_rear_right_joint");
  joint_limits_interface::VelocityJointSoftLimitsHandle rear_right_handle(
    joint_handle,
    limits,
    soft_limits
  );
  joint_limits_interface_.registerHandle(rear_right_handle);

  joint_handle  = joint_vel_interface_.getHandle("wheel_rear_left_joint");
  joint_limits_interface::VelocityJointSoftLimitsHandle rear_left_handle(
    joint_handle,
    limits,
    soft_limits
  );
  joint_limits_interface_.registerHandle(rear_left_handle);
}

void Omnibot::read() {
  bool open_loop_ = true;

  if(open_loop_) {
    front_left_vel_ = lowpass_front_left_cmd_;
    front_right_vel_ = lowpass_front_right_cmd_;
    rear_left_vel_ = lowpass_rear_left_cmd_;
    rear_right_vel_ = lowpass_rear_right_cmd_;
  } else {
    //TODO: get vel from ecoders
  }

  front_left_pos_ += front_left_vel_*getPeriod().toSec();
  front_right_pos_ += front_right_vel_*getPeriod().toSec();
  rear_left_pos_ += rear_left_vel_*getPeriod().toSec();
  rear_right_pos_ += rear_right_vel_*getPeriod().toSec();
}

void Omnibot::write() {
  joint_limits_interface_.enforceLimits(getPeriod());

  if(use_lowpass_){
    lowPassJoints();
  } else {
    lowpass_front_left_cmd_ = front_left_cmd_;
    lowpass_front_right_cmd_ = front_right_cmd_;
    lowpass_rear_left_cmd_ = rear_left_cmd_;
    lowpass_rear_right_cmd_ = rear_right_cmd_;
  }

  //TODO: send wheel rotational velocities to arduino
}

void Omnibot::lowPassJoints() {
  double min_vel_ = 0.001;

  if(fabs(front_left_cmd_) > min_vel_) {
    lowpass_front_left_cmd_ = lowPassFilter(
      front_left_cmd_,
      last_front_left_cmd_,
      getPeriod().toSec(),
      lowpass_constant_
    );
  } else {
    lowpass_front_left_cmd_ = 0.0;
  }
  last_front_left_cmd_ = lowpass_front_left_cmd_;

  if(fabs(front_right_cmd_) > min_vel_) {
    lowpass_front_right_cmd_ = lowPassFilter(
      front_right_cmd_,
      last_front_right_cmd_,
      getPeriod().toSec(),
      lowpass_constant_
    );
  } else {
    lowpass_front_right_cmd_ = 0.0;
  }
  last_front_right_cmd_ = lowpass_front_right_cmd_;

  if(fabs(rear_left_cmd_) > min_vel_) {
    lowpass_rear_left_cmd_ = lowPassFilter(
      rear_left_cmd_,
      last_rear_left_cmd_,
      getPeriod().toSec(),
      lowpass_constant_
    );
  } else {
    lowpass_rear_left_cmd_ = 0.0;
  }
  last_rear_left_cmd_ = lowpass_rear_left_cmd_;

  if(fabs(rear_right_cmd_) > min_vel_) {
    lowpass_rear_right_cmd_ = lowPassFilter(
      rear_right_cmd_,
      last_rear_right_cmd_,
      getPeriod().toSec(),
      lowpass_constant_
    );
  } else {
    lowpass_rear_right_cmd_ = 0.0;
  }
  last_rear_right_cmd_ = lowpass_rear_right_cmd_;
}

double Omnibot::lowPassFilter(double x, double y0, double dt, double T)
{
  double res = y0 + (x - y0) * (dt/(dt+T));
  return res;
}

}; // namespace omnibot_driver
