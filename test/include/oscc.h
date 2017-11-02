#ifndef _OSCC_H
#define _OSCC_H

#include <linux/can.h>

#include "brake_can_protocol.h"
#include "fault_can_protocol.h"
#include "steering_can_protocol.h"
#include "throttle_can_protocol.h"

typedef enum { OSCC_OK, OSCC_ERROR, OSCC_WARNING } oscc_result_t;

oscc_result_t oscc_publish_steering_torque(double input);

oscc_result_t oscc_publish_brake_position(double input);

oscc_result_t oscc_publish_throttle_position(double input);

oscc_result_t oscc_enable(void);

oscc_result_t oscc_disable(void);

oscc_result_t oscc_subscribe_to_brake_reports(void (*callback)(oscc_brake_report_s* report));

oscc_result_t oscc_subscribe_to_throttle_reports(void (*callback)(oscc_throttle_report_s* report));

oscc_result_t oscc_subscribe_to_steering_reports(void (*callback)(oscc_steering_report_s* report));

oscc_result_t oscc_subscribe_to_fault_reports(void (*callback)(oscc_fault_report_s* report));

oscc_result_t oscc_subscribe_to_obd_messages(void (*callback)(struct can_frame* frame));

// Expose the Callback for testing API
void (*steering_report_callback)(oscc_steering_report_s* report);

void (*brake_report_callback)(oscc_brake_report_s* report);

void (*throttle_report_callback)(oscc_throttle_report_s* report);

void (*fault_report_callback)(oscc_fault_report_s* report);

void (*obd_frame_callback)(struct can_frame* frame);

// Expose variables that are set for validating API call
double get_brake_command();

double get_steering_torque();

double get_throttle_command();

int get_enable_disable();

#endif /* _OSCC_H */
