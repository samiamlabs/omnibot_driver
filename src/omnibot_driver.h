#ifndef OMNIBOT_DRIVER_H
#define OMNIBOT_DRIVER_H

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include "CmdMessenger.h"

namespace omnibot_driver
{
  enum Command {
    CMD_STEER = 0
  };

class Omnibot : public hardware_interface::RobotHW
{
public:
  Omnibot();
  ~Omnibot() {}

  void registerJoints();
  void registerJointLimits();

  void read();
  void write();

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.02);}

  double getFrontRightCmd() {return lowpass_front_right_cmd_;}
  double getFrontLeftCmd() {return lowpass_front_left_cmd_;}
  double getRearRightCmd() {return lowpass_rear_right_cmd_;}
  double getRearLeftCmd() {return lowpass_rear_left_cmd_;}

private:
  double lowPassFilter(double x, double y0, double dt, double T);
  void lowPassJoints();

  bool use_lowpass_;
  double lowpass_constant_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface joint_vel_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface joint_limits_interface_;

  double front_right_cmd_, front_right_pos_, front_right_vel_, front_right_eff_;
  double front_left_cmd_, front_left_pos_, front_left_vel_, front_left_eff_;
  double rear_right_cmd_, rear_right_pos_, rear_right_vel_, rear_right_eff_;
  double rear_left_cmd_, rear_left_pos_, rear_left_vel_, rear_left_eff_;

  double last_front_right_cmd_, lowpass_front_right_cmd_;
  double last_front_left_cmd_, lowpass_front_left_cmd_;
  double last_rear_right_cmd_, lowpass_rear_right_cmd_;
  double last_rear_left_cmd_, lowpass_rear_left_cmd_;

  serial::Serial serial_;
  CmdMessenger cmd_messenger_;
};

};
#endif
