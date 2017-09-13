#ifndef ROS_TO_OSCC_H
#define ROS_TO_OSCC_H

#include <signal.h>

extern "C" {
#include <oscc.h>
}

#include <ros/ros.h>

#include <roscco/BrakeCommand.h>
#include <roscco/SteeringCommand.h>
#include <roscco/ThrottleCommand.h>
#include <roscco/EnableDisable.h>

class RosToOscc
{
public:
  RosToOscc(ros::NodeHandle* public_nh, ros::NodeHandle* private_nh);

  void brakeCommandCallback(
      const roscco::BrakeCommand::ConstPtr& msg);

  void steeringCommandCallback(
      const roscco::SteeringCommand::ConstPtr& msg);

  void throttleCommandCallback(
      const roscco::ThrottleCommand::ConstPtr& msg);

  void enableDisableCallback(
      const roscco::EnableDisable::ConstPtr& msg);

private:
  ros::Subscriber topic_brake_command_;

  ros::Subscriber topic_steering_command_;

  ros::Subscriber topic_throttle_command_;

  ros::Subscriber topic_enable_disable_command_;
};

#endif // ROS_TO_OSCC_H
