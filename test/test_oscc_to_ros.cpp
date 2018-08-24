#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/gtest.h>

#include <ros/ros.h>

#include <roscco/oscc_to_ros.h>

// Time to allow ROS to process callbacks and publish a message
const double SLEEP_TIME = 0.05;

template <class T>
class MessageHelper
{
public:
  MessageHelper() : count(0)
  {
  }

  void cb(const T& msg)
  {
    ++count;
    message = &msg;
  }

  const T* message;
  uint32_t count;
};

RC_GTEST_PROP(TestOSCCToROS, CastBrakeReport, ())
{
  ros::NodeHandle sub_nh;
  MessageHelper<roscco::BrakeReport> h;
  ros::Subscriber sub = sub_nh.subscribe("brake_report", 0, &MessageHelper<roscco::BrakeReport>::cb, &h);

  oscc_brake_report_s* data = new oscc_brake_report_s;

  data->magic[0] = *rc::gen::arbitrary<uint8_t>();
  data->magic[1] = *rc::gen::arbitrary<uint8_t>();
  data->enabled = *rc::gen::arbitrary<uint8_t>();
  data->operator_override = *rc::gen::arbitrary<uint8_t>();
  data->dtcs = *rc::gen::arbitrary<uint8_t>();
  data->reserved[0] = *rc::gen::arbitrary<uint8_t>();
  data->reserved[1] = *rc::gen::arbitrary<uint8_t>();
  data->reserved[2] = *rc::gen::arbitrary<uint8_t>();

  brake_report_callback(data);

  ros::WallDuration(SLEEP_TIME).sleep();
  RC_ASSERT(h.count > 0);
  RC_ASSERT(h.message->data.magic[0] == data->magic[0]);
  RC_ASSERT(h.message->data.magic[1] == data->magic[1]);
  RC_ASSERT(h.message->data.enabled == data->enabled);
  RC_ASSERT(h.message->data.operator_override == data->operator_override);
  RC_ASSERT(h.message->data.dtcs == data->dtcs);
  RC_ASSERT(h.message->data.reserved[0] == data->reserved[0]);
  RC_ASSERT(h.message->data.reserved[1] == data->reserved[1]);
  RC_ASSERT(h.message->data.reserved[2] == data->reserved[2]);

  delete data;
}

RC_GTEST_PROP(TestOSCCToROS, CastThrottleReport, ())
{
  ros::NodeHandle sub_nh;
  MessageHelper<roscco::ThrottleReport> h;
  ros::Subscriber sub = sub_nh.subscribe("throttle_report", 0, &MessageHelper<roscco::ThrottleReport>::cb, &h);

  oscc_throttle_report_s* data = new oscc_throttle_report_s;

  data->magic[0] = *rc::gen::arbitrary<uint8_t>();
  data->magic[1] = *rc::gen::arbitrary<uint8_t>();
  data->enabled = *rc::gen::arbitrary<uint8_t>();
  data->operator_override = *rc::gen::arbitrary<uint8_t>();
  data->dtcs = *rc::gen::arbitrary<uint8_t>();
  data->reserved[0] = *rc::gen::arbitrary<uint8_t>();
  data->reserved[1] = *rc::gen::arbitrary<uint8_t>();
  data->reserved[2] = *rc::gen::arbitrary<uint8_t>();

  throttle_report_callback(data);

  ros::WallDuration(SLEEP_TIME).sleep();
  RC_ASSERT(h.count > 0);
  RC_ASSERT(h.message->data.magic[0] == data->magic[0]);
  RC_ASSERT(h.message->data.magic[1] == data->magic[1]);
  RC_ASSERT(h.message->data.enabled == data->enabled);
  RC_ASSERT(h.message->data.operator_override == data->operator_override);
  RC_ASSERT(h.message->data.dtcs == data->dtcs);
  RC_ASSERT(h.message->data.reserved[0] == data->reserved[0]);
  RC_ASSERT(h.message->data.reserved[1] == data->reserved[1]);
  RC_ASSERT(h.message->data.reserved[2] == data->reserved[2]);

  delete data;
}

RC_GTEST_PROP(TestOSCCToROS, CastSteeringReport, ())
{
  ros::NodeHandle sub_nh;
  MessageHelper<roscco::SteeringReport> h;
  ros::Subscriber sub = sub_nh.subscribe("steering_report", 0, &MessageHelper<roscco::SteeringReport>::cb, &h);

  oscc_steering_report_s* data = new oscc_steering_report_s;

  data->magic[0] = *rc::gen::arbitrary<uint8_t>();
  data->magic[1] = *rc::gen::arbitrary<uint8_t>();
  data->enabled = *rc::gen::arbitrary<uint8_t>();
  data->operator_override = *rc::gen::arbitrary<uint8_t>();
  data->dtcs = *rc::gen::arbitrary<uint8_t>();
  data->reserved[0] = *rc::gen::arbitrary<uint8_t>();
  data->reserved[1] = *rc::gen::arbitrary<uint8_t>();
  data->reserved[2] = *rc::gen::arbitrary<uint8_t>();

  steering_report_callback(data);

  ros::WallDuration(SLEEP_TIME).sleep();
  RC_ASSERT(h.count > 0);
  RC_ASSERT(h.message->data.magic[0] == data->magic[0]);
  RC_ASSERT(h.message->data.magic[1] == data->magic[1]);
  RC_ASSERT(h.message->data.enabled == data->enabled);
  RC_ASSERT(h.message->data.operator_override == data->operator_override);
  RC_ASSERT(h.message->data.dtcs == data->dtcs);
  RC_ASSERT(h.message->data.reserved[0] == data->reserved[0]);
  RC_ASSERT(h.message->data.reserved[1] == data->reserved[1]);
  RC_ASSERT(h.message->data.reserved[2] == data->reserved[2]);

  delete data;
}

RC_GTEST_PROP(TESTOSCCToROS, CastFaultReport, ())
{
  ros::NodeHandle sub_nh;
  MessageHelper<roscco::FaultReport> h;
  ros::Subscriber sub = sub_nh.subscribe("fault_report", 0, &MessageHelper<roscco::FaultReport>::cb, &h);

  oscc_fault_report_s* data = new oscc_fault_report_s;

  data->magic[0] = *rc::gen::arbitrary<uint8_t>();
  data->magic[1] = *rc::gen::arbitrary<uint8_t>();
  data->fault_origin_id = *rc::gen::arbitrary<uint32_t>();
  data->dtcs = *rc::gen::arbitrary<uint8_t>();
  data->reserved = *rc::gen::arbitrary<uint8_t>();

  fault_report_callback(data);

  ros::WallDuration(SLEEP_TIME).sleep();

  RC_ASSERT(h.count > 0);
  RC_ASSERT(h.message->data.magic[0] == data->magic[0]);
  RC_ASSERT(h.message->data.magic[1] == data->magic[1]);
  RC_ASSERT(h.message->data.fault_origin_id == data->fault_origin_id);
  RC_ASSERT(h.message->data.dtcs == data->dtcs);
  RC_ASSERT(h.message->data.reserved == data->reserved);

  delete data;
}

RC_GTEST_PROP(TESTOSCCToROS, CastOBDFrame, ())
{
  ros::NodeHandle sub_nh;
  MessageHelper<roscco::CanFrame> h;
  ros::Subscriber sub = sub_nh.subscribe("can_frame", 0, &MessageHelper<roscco::CanFrame>::cb, &h);

  can_frame* frame = new can_frame;

  frame->can_id = *rc::gen::arbitrary<uint32_t>();
  frame->can_dlc = 8;

  for (int i = 0; i < frame->can_dlc; i++)
  {
    frame->data[i] = *rc::gen::arbitrary<uint8_t>();
  }

  obd_frame_callback(frame);

  ros::WallDuration(SLEEP_TIME).sleep();

  RC_ASSERT(h.count > 0);

  RC_ASSERT(h.message->frame.can_id == frame->can_id);
  RC_ASSERT(h.message->frame.can_dlc == frame->can_dlc);

  for (int i = 0; i < frame->can_dlc; i++)
  {
    RC_ASSERT(h.message->frame.data[i] == frame->data[i]);
  }

  delete frame;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_oscc_to_ros");

  ros::NodeHandle n, nh("~");

  OsccToRos oscc_publisher(&n, &nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int ret = RUN_ALL_TESTS();

  spinner.stop();
  ros::shutdown();
  return ret;
}
