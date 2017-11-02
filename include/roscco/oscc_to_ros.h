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
  /**
   * @brief OSCCToRos class initializer
   *
   * This function constructs ROS publishers and initializes the OSCC callbacks to begin transfering messages from OSCC
   * to ROS
   *
   * @param public_nh  The public node handle to use for ROS publishers.
   * @param private_nh The private node handle for ROS parameters.
   */
  OsccToRos(ros::NodeHandle* public_nh, ros::NodeHandle* private_nh);

  /**
   * @brief Callback function to publish oscc_steering_report messages on ROS.
   *
   * This function is a callback that consumes oscc_steering_report messages by utilizing the OSCC callback and
   * publishes that message with a ROS Publisher as a SteeringReport mesage.
   *
   * @param report The osc_steering_report message to be consumed by ROS.
   */
  static void steering_callback(oscc_steering_report_s* report);

  /**
   * @brief Callback function to publish oscc_brake_report messages on ROS.
   *
   * This function is a callback that consumes oscc_brake_report messages by utilizing the OSCC callback and
   * publishes that message with a ROS Publisher as a BrakeReport message.
   *
   * @param report The oscc_brake_report message to be consumed by ROS.
   */
  static void brake_callback(oscc_brake_report_s* report);

  /**
   * @brief Callback function to publish oscc_throttle_report messages on ROS.
   *
   * This function is a callback that consumes oscc_throttle_report messages by utilizing the OSCC callback and
   * publishes that message with a ROS Publisher as a ThrottleReport message.
   *
   * @param report The oscc_throttle_report message to be consumed by ROS.
   */
  static void throttle_callback(oscc_throttle_report_s* report);

  /**
   * @brief Callback function to publish oscc_fault_report messages on ROS.
   *
   * This function is a callback that consumes oscc_fault_report messages by utilizing the OSCC callback and publishes
   * that message with a ROS Publisher as a FaultReport message.
   *
   * @param report The oscc_fault_report message to be consumed by ROS.
   */
  static void fault_callback(oscc_fault_report_s* report);

  /**
   * @brief Callback function to publish can_frame messages on ROS.
   *
   * This function is a callback that consumes can_frame messages by utilizing the OSCC callback and publishes that
   * message with a ROS Publisher as a CanFrame message.
   *
   * @param report The oscc can_frame message to be consumed by ROS.
   */
  static void obd_callback(can_frame* frame);

  /**
   * @brief Cast OSCC report to ROS message and publish
   *
   * This function casts an oscc report type to a ros message and publishes the message onto ROS. Where the ROSMSGTYPE
   * is the type that gets published containing the ROS header and wraps the now casted ROSDATATYPE. The OSCCTYPE
   * is cast to the ROSDATATYPE which should be the same memory layout as the OSCCTYPE.
   *
   * Usage:
   *
   *  cast an oscc_steering_report message to ROS and publish with the Publisher topic_steering_report_.
   *
   * cast_callback<oscc_steering_report_s,
   *                roscco::SteeringReport,
   *                roscco::SteeringReportData>(report, &topic_steering_report_);
   *
   * @tparam OSCCTYPE    OSCC report message data type
   * @tparam ROSMSGTYPE  ROS parent message type
   * @tparam ROSDATATYPE ROS data message type
   *
   * @param report       OSCC report message
   * @param pub          ROS publisher to send the oscc message
   */
  template <class OSCCTYPE, class ROSMSGTYPE, class ROSDATATYPE>
  static void cast_callback(OSCCTYPE* report, ros::Publisher* pub);
};

#endif  // OSCC_TO_ROS_H
