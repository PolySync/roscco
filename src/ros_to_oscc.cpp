#include <roscco/ros_to_oscc.h>

RosToOscc::RosToOscc(ros::NodeHandle* public_nh, ros::NodeHandle* private_nh)
{
  sigset_t mask;
  sigset_t orig_mask;

  sigemptyset(&mask);
  sigemptyset(&orig_mask);
  sigaddset(&mask, SIGIO);

  // Temporary block of OSCC SIGIO while initializing ROS publication to prevent
  // signal conflicts
  if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0)
  {
    ROS_ERROR("Failed to block SIGIO");
  }

  topic_brake_command_ =
      public_nh->subscribe<roscco::BrakeCommand>("brake_command", 10, &RosToOscc::brakeCommandCallback, this);

  topic_steering_command_ =
      public_nh->subscribe<roscco::SteeringCommand>("steering_command", 10, &RosToOscc::steeringCommandCallback, this);

  topic_throttle_command_ =
      public_nh->subscribe<roscco::ThrottleCommand>("throttle_command", 10, &RosToOscc::throttleCommandCallback, this);

  topic_enable_disable_command_ =
      public_nh->subscribe<roscco::EnableDisable>("enable_disable", 10, &RosToOscc::enableDisableCallback, this);

  if (sigprocmask(SIG_SETMASK, &orig_mask, NULL) < 0)
  {
    ROS_ERROR("Failed to unblock SIGIO");
  }
};

void RosToOscc::brakeCommandCallback(const roscco::BrakeCommand::ConstPtr& msg)
{
  oscc_result_t ret = OSCC_ERROR;

  ret = oscc_publish_brake_position(msg->brake_position);

  if (ret == OSCC_ERROR)
  {
    ROS_ERROR("OSCC_ERROR occured while trying send the brake position.");
  }
  else if (ret == OSCC_WARNING)
  {
    ROS_WARN("OSCC_WARNING occured while trying send the brake position.");
  }
};

void RosToOscc::steeringCommandCallback(const roscco::SteeringCommand::ConstPtr& msg)
{
  oscc_result_t ret = OSCC_ERROR;

  ret = oscc_publish_steering_torque(msg->steering_torque);

  if (ret == OSCC_ERROR)
  {
    ROS_ERROR("OSCC_ERROR occured while trying send the steering torque.");
  }
  else if (ret == OSCC_WARNING)
  {
    ROS_WARN("OSCC_WARNING occured while trying send the steering torque.");
  }
};

void RosToOscc::throttleCommandCallback(const roscco::ThrottleCommand::ConstPtr& msg)
{
  oscc_result_t ret = OSCC_ERROR;

  ret = oscc_publish_throttle_position(msg->throttle_position);

  if (ret == OSCC_ERROR)
  {
    ROS_ERROR("OSCC_ERROR occured while trying send the throttle position.");
  }
  else if (ret == OSCC_WARNING)
  {
    ROS_WARN("OSCC_WARNING occured while trying send the throttle position.");
  }
};

void RosToOscc::enableDisableCallback(const roscco::EnableDisable::ConstPtr& msg)
{
  oscc_result_t ret = OSCC_ERROR;

  ret = msg->enable_control ? oscc_enable() : oscc_disable();

  if (ret == OSCC_ERROR)
  {
    ROS_ERROR("OSCC_ERROR occured while trying to enable or disable control.");
  }
  else if (ret == OSCC_WARNING)
  {
    ROS_WARN("OSCC_WARNING occured while trying to enable or disable control.");
  }
}
