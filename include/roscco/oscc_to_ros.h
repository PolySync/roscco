#ifndef OSCC_TO_ROS_H
#define OSCC_TO_ROS_H

#include <signal.h>

extern "C" {
#include <oscc.h>
}

#include <ros/ros.h>

#include <roscco/BrakeReport.h>
#include <roscco/BrakeReportData.h>
#include <roscco/CanFrame.h>
#include <roscco/FaultReport.h>
#include <roscco/FaultReportData.h>
#include <roscco/SteeringReport.h>
#include <roscco/SteeringReportData.h>
#include <roscco/ThrottleReport.h>
#include <roscco/ThrottleReportData.h>

static ros::Publisher topic_brake_report_;

static ros::Publisher topic_steering_report_;

static ros::Publisher topic_throttle_report_;

static ros::Publisher topic_fault_report_;

static ros::Publisher topic_obd_messages_;

class OsccToRos
{
public:
  OsccToRos(ros::NodeHandle* public_nh, ros::NodeHandle* private_nh);

  template <class OSCCTYPE, class ROSTYPE, class ROSSUBTYPE>
  static void cast_callback(OSCCTYPE* report, ros::Publisher* pub);

  static void steering_callback(oscc_steering_report_s* report);

  static void brake_callback(oscc_brake_report_s* report);

  static void throttle_callback(oscc_throttle_report_s* report);

  static void fault_callback(oscc_fault_report_s* report);

  static void obd_callback(can_frame* frame);
};

#endif  // OSCC_TO_ROS_H
