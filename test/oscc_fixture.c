#include <oscc.h>

static double steering_torque;
static double brake_command;
static double throttle_command;
int enabled;

double get_brake_command() { return brake_command; }

double get_steering_torque() { return steering_torque; }

double get_throttle_command() { return throttle_command; }

int get_enable_disable() { return enabled; }

oscc_result_t oscc_publish_steering_torque(double input)
{
  steering_torque = input;

  return OSCC_OK;
}

oscc_result_t oscc_publish_brake_position(double input)
{
  brake_command = input;

  return OSCC_OK;
}

oscc_result_t oscc_publish_throttle_position(double input)
{
  throttle_command = input;

  return OSCC_OK;
}

oscc_result_t oscc_enable(void)
{
  ++enabled;

  return OSCC_OK;
}

oscc_result_t oscc_disable(void)
{
  enabled = 0;

  return OSCC_OK;
}

oscc_result_t
oscc_subscribe_to_brake_reports(void (*callback)(oscc_brake_report_s* report))
{
  brake_report_callback = callback;
  return OSCC_OK;
}

oscc_result_t oscc_subscribe_to_throttle_reports(
    void (*callback)(oscc_throttle_report_s* report))
{
  throttle_report_callback = callback;
  return OSCC_OK;
}

oscc_result_t oscc_subscribe_to_steering_reports(
    void (*callback)(oscc_steering_report_s* report))
{
  steering_report_callback = callback;
  return OSCC_OK;
}

oscc_result_t
oscc_subscribe_to_fault_reports(void (*callback)(oscc_fault_report_s* report))
{
  fault_report_callback = callback;
  return OSCC_OK;
}

oscc_result_t
oscc_subscribe_to_obd_messages(void (*callback)(struct can_frame* frame))
{
  obd_frame_callback = callback;
  return OSCC_OK;
}
