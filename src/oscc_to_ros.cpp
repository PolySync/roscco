#include <roscco/oscc_to_ros.h>

OsccToRos::OsccToRos(ros::NodeHandle* public_nh, ros::NodeHandle* private_nh)
{
  sigset_t mask;
  sigset_t orig_mask;

  sigemptyset(&mask);
  sigemptyset(&orig_mask);
  sigaddset(&mask, SIGIO);

  // Temporary block of OSCC SIGIO while initializing ROS subscription to
  // prevent signal conflicts
  if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0)
  {
    ROS_ERROR("Failed to block SIGIO");
  }

  topic_brake_report_ = public_nh->advertise<roscco::BrakeReport>("brake_report", 10);

  topic_steering_report_ = public_nh->advertise<roscco::SteeringReport>("steering_report", 10);

  topic_throttle_report_ = public_nh->advertise<roscco::ThrottleReport>("throttle_report", 10);

  topic_fault_report_ = public_nh->advertise<roscco::FaultReport>("fault_report", 10);

  topic_obd_messages_ = public_nh->advertise<roscco::CanFrame>("can_frame", 10);

  if (sigprocmask(SIG_SETMASK, &orig_mask, NULL) < 0)
  {
    ROS_ERROR("Failed to unblock SIGIO");
  }

  oscc_subscribe_to_brake_reports(brake_callback);

  oscc_subscribe_to_steering_reports(steering_callback);

  oscc_subscribe_to_throttle_reports(throttle_callback);

  oscc_subscribe_to_fault_reports(fault_callback);

  oscc_subscribe_to_obd_messages(obd_callback);
}

void OsccToRos::steering_callback(oscc_steering_report_s* report)
{
  cast_callback<oscc_steering_report_s, roscco::SteeringReport, roscco::SteeringReportData>(report,
                                                                                            &topic_steering_report_);
}

void OsccToRos::brake_callback(oscc_brake_report_s* report)
{
  cast_callback<oscc_brake_report_s, roscco::BrakeReport, roscco::BrakeReportData>(report, &topic_brake_report_);
}

void OsccToRos::throttle_callback(oscc_throttle_report_s* report)
{
  cast_callback<oscc_throttle_report_s, roscco::ThrottleReport, roscco::ThrottleReportData>(report,
                                                                                            &topic_throttle_report_);
}

template <class OSCCTYPE, class ROSMSGTYPE, class ROSDATATYPE>
void OsccToRos::cast_callback(OSCCTYPE* report, ros::Publisher* pub)
{
  ROSMSGTYPE* ros_message(new ROSMSGTYPE);

  ROSDATATYPE* data = (ROSDATATYPE*)report;

  ros_message->data = *data;

  ros_message->header.stamp = ros::Time::now();

  pub->publish(*ros_message);

  delete ros_message;
}

void OsccToRos::fault_callback(oscc_fault_report_s* report)
{
  roscco::FaultReport* ros_message(new roscco::FaultReport);

  // ROS does not pack the structs so individual assignment is required over
  // cast
  ros_message->data.magic[0] = report->magic[0];
  ros_message->data.magic[1] = report->magic[1];
  ros_message->data.fault_origin_id = report->fault_origin_id;
  ros_message->data.reserved[0] = report->reserved[0];
  ros_message->data.reserved[1] = report->reserved[1];

  ros_message->header.stamp = ros::Time::now();

  topic_fault_report_.publish(*ros_message);

  delete ros_message;
}

void OsccToRos::obd_callback(can_frame* frame)
{
  roscco::CanFrame* ros_message(new roscco::CanFrame);

  roscco::CanFrameData* data = (roscco::CanFrameData*)frame;

  ros_message->frame = *data;

  ros_message->header.stamp = ros::Time::now();

  topic_obd_messages_.publish(*ros_message);

  delete ros_message;
}
