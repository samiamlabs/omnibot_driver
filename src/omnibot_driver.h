#ifndef OMNIBOT_DRIVER_H
#define OMNIBOT_DRIVER_H

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace omnibot_driver
{

class Omnibot : public hardware_interface::RobotHW
{
public:
  Omnibot();
  ~Omnibot() {}

  void read();
  void write();

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.02);}

private:
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface joint_vel_interface_;

  double front_right_cmd_, front_right_pos_, front_right_vel_, front_right_eff_;
  double front_left_cmd_, front_left_pos_, front_left_vel_, front_left_eff_;
  double rear_right_cmd_, rear_right_pos_, rear_right_vel_, rear_right_eff_;
  double rear_left_cmd_, rear_left_pos_, rear_left_vel_, rear_left_eff_;
};

};
#endif
