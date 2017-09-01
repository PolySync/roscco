#include <string>

extern "C" {
#include <oscc.h>
}

#include <ros/ros.h>

#include <roscco/ros_to_oscc.h>
#include <roscco/oscc_to_ros.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "roscco_node");

  ros::NodeHandle public_nh;
  ros::NodeHandle private_nh("~");

  int can_channel;
  private_nh.param<int>("can_channel", can_channel, 0);

  oscc_result_t ret = OSCC_ERROR;

  ret = oscc_open( can_channel );

  if(ret != OSCC_OK )
  {
    ROS_ERROR("Could not initialize OSCC");
  }

  RosToOscc subcriber(&public_nh, &private_nh);
  OsccToRos publisher(&public_nh, &private_nh);

  ros::spin();

  ret = oscc_close( 0 );

  ros::waitForShutdown();
}
