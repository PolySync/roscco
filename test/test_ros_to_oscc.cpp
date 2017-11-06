#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>
#include <ros/ros.h>

#include <can_protocols/brake_can_protocol.h>
#include <can_protocols/steering_can_protocol.h>
#include <can_protocols/throttle_can_protocol.h>

#include <roscco/ros_to_oscc.h>

#include <vehicles.h>

// Validates that publishing a brake command message yields the expected API
// call to OSCC
RC_GTEST_PROP(TestOSCCToROS, SendBrakeCommandMessage, ())
{
  ros::NodeHandle pub_nh;

  ros::Publisher pub = pub_nh.advertise<roscco::BrakeCommand>("brake_command", 10);

  roscco::BrakeCommand* command(new roscco::BrakeCommand);

  command->brake_position = *rc::gen::arbitrary<double>();

  command->header.stamp = ros::Time::now();

  pub.publish(*command);

  ros::WallDuration(0.1).sleep();

  RC_ASSERT(get_brake_command() == command->brake_position);
}

// Validates that publishing a steering command message yields the expected API
// call to OSCC
RC_GTEST_PROP(TestOSCCToROS, SendSteeringCommandMessage, ())
{
  ros::NodeHandle pub_nh;

  ros::Publisher pub = pub_nh.advertise<roscco::SteeringCommand>("steering_command", 10);

  roscco::SteeringCommand* command(new roscco::SteeringCommand);

  command->steering_torque = *rc::gen::arbitrary<double>();

  command->header.stamp = ros::Time::now();

  pub.publish(*command);

  ros::WallDuration(0.1).sleep();

  // TODO: Should there be bounds checking in ROS for these?
  RC_ASSERT(get_steering_torque() == command->steering_torque);
}

// Validates that publishing a steering command message yields the expected API
// call to OSCC
RC_GTEST_PROP(TestOSCCToROS, SendThrottleCommandMessage, ())
{
  ros::NodeHandle pub_nh;

  ros::Publisher pub = pub_nh.advertise<roscco::ThrottleCommand>("throttle_command", 10);

  roscco::ThrottleCommand* command(new roscco::ThrottleCommand);

  command->throttle_position = *rc::gen::arbitrary<double>();

  command->header.stamp = ros::Time::now();

  pub.publish(*command);

  ros::WallDuration(0.1).sleep();

  // TODO: Should there be bounds checking in ROS for these?
  RC_ASSERT(get_throttle_command() == command->throttle_position);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_ros_to_oscc");

  ros::NodeHandle public_nh, private_nh("~");

  int ret = OSCC_ERROR;

  RosToOscc subcriber(&public_nh, &private_nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ret = RUN_ALL_TESTS();

  spinner.stop();
  ros::shutdown();

  return ret;
}
