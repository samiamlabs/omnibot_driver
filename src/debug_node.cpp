#include <ros/ros.h>

// #include <std_srvs/Empty.h>
// #include <std_msgs/Int64.h>

#include <std_msgs/Float64.h>
#include <string.h>
#include <controller_manager/controller_manager.h>

#include "omnibot_driver.cpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tricycle_driver");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // ROS_INFO_STREAM("period: " << robot.getPeriod().toSec());
  omnibot_driver::Omnibot robot;
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  // ros::Rate rate(0.1);
  ROS_ERROR("init debug...");

  ros::Publisher front_right_cmd_pub = nh.advertise<std_msgs::Float64>("front_right_cmd", 1);
  ros::Publisher front_left_cmd_pub = nh.advertise<std_msgs::Float64>("front_left_cmd", 1);
  ros::Publisher rear_right_cmd_pub = nh.advertise<std_msgs::Float64>("rear_right_cmd", 1);
  ros::Publisher rear_left_cmd_pub = nh.advertise<std_msgs::Float64>("rear_left_cmd", 1);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std_msgs::Float64 msg;

  while(ros::ok())
  {
    // ROS_ERROR("debug loop...");

    robot.read();
    cm.update(robot.getTime(), robot.getPeriod());
    robot.write();

    // Publish on ros topic
    msg.data = robot.getFrontRightCmd();
    front_right_cmd_pub.publish(msg);
    msg.data = robot.getFrontLeftCmd();
    front_left_cmd_pub.publish(msg);
    msg.data = robot.getRearRightCmd();
    rear_right_cmd_pub.publish(msg);
    msg.data = robot.getRearLeftCmd();
    rear_left_cmd_pub.publish(msg);

    rate.sleep();
  }

}
