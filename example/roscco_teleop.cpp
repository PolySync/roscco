#include <geometry_msgs/Accel.h>
#include <ros/ros.h>
#include <roscco/BrakeCommand.h>
#include <roscco/EnableDisable.h>
#include <roscco/SteeringCommand.h>
#include <roscco/ThrottleCommand.h>
#include <sensor_msgs/Joy.h>

double calc_exponential_average(double average, double setpoint, double factor);
double linear_tranformation(double value, double high_1, double low_1, double high_2, double low_2);

class RosccoTeleop
{
public:
  RosccoTeleop();

private:
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void timerCallback(const ros::TimerEvent& event);

  ros::NodeHandle nh_;

  ros::Publisher throttle_pub_;
  ros::Publisher brake_pub_;
  ros::Publisher steering_pub_;
  ros::Publisher enable_disable_pub_;
  ros::Subscriber joy_sub_;
  ros::Timer timer_;

  int previous_start_state_ = 0;
  int previous_back_state_ = 0;

  // Number of messages to retain when the message queue is full
  const int queue_size = 10;

  // Timed callback frequency set to OSCC recommended publishing rate of 20 Hz (50 ms == 0.05 s)
  const float callback_freq = 0.05;  // Units in Seconds

  // OSCC input range
  const double brake_max_ = 1;
  const double brake_min_ = 0;
  const double throttle_max_ = 1;
  const double throttle_min_ = 0;
  const double steering_max_ = 1;
  const double steering_min_ = -1;

  // Store last known value for timed callback
  double brake_ = 0.0;
  double throttle_ = 0.0;
  double steering_ = 0.0;
  bool enabled_ = false;

  // Smooth the steering to remove twitchy joystick movements
  const double data_smoothing_factor = 0.1;
  double steering_average_ = 0.0;

  // Variable to ensure joystick triggers have been initialized
  bool initialized_ = false;

  // The threshold for considering the controller triggers to be parked in the correct position
  const double parked_threshold_ = 0.99;

  const int brake_axes_ = 2;
  const int throttle_axes_ = 5;
  const int steering_axes_ = 0;
  const int start_button_ = 7;
  const int back_button_ = 6;

  const double trigger_min_ = 1;
  const double trigger_max_ = -1;
  const double joystick_min_ = 1;
  const double joystick_max_ = -1;
};

RosccoTeleop::RosccoTeleop()
{
  brake_pub_ = nh_.advertise<roscco::BrakeCommand>("brake_command", queue_size);
  throttle_pub_ = nh_.advertise<roscco::ThrottleCommand>("throttle_command", queue_size);
  steering_pub_ = nh_.advertise<roscco::SteeringCommand>("steering_command", queue_size);
  enable_disable_pub_ = nh_.advertise<roscco::EnableDisable>("enable_disable", queue_size);

  // Timed callback to ensure publishing to OSCC < 200 ms
  timer_ = nh_.createTimer(ros::Duration(callback_freq), &RosccoTeleop::timerCallback, this);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", queue_size, &RosccoTeleop::joystickCallback, this);
}

void RosccoTeleop::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // gamepad triggers default 0 prior to using them which is 50% for the logitech and xbox controller the initilization
  // is to ensure the triggers have been pulled prior to enabling OSCC command
  if (initialized_)
  {
    // Map the trigger values [1, -1] to oscc values [0, 1]
    brake_ = linear_tranformation(joy->axes[brake_axes_], trigger_max_, trigger_min_, brake_max_, brake_min_);
    throttle_ =
        linear_tranformation(joy->axes[throttle_axes_], trigger_max_, trigger_min_, throttle_max_, throttle_min_);

    // Map the joystick to steering [1, -1] to oscc values [-1, 1]
    steering_ =
        linear_tranformation(joy->axes[steering_axes_], trigger_max_, trigger_min_, steering_max_, steering_min_);

    roscco::EnableDisable enable_msg;
    enable_msg.header.stamp = ros::Time::now();

    if ((previous_back_state_ == 0) && joy->buttons[back_button_])
    {
      enable_msg.enable_control = false;
      enable_disable_pub_.publish(enable_msg);
      enabled_ = false;
    }
    else if ((previous_start_state_ == 0) && joy->buttons[start_button_])
    {
      enable_msg.enable_control = true;
      enable_disable_pub_.publish(enable_msg);
      enabled_ = true;
    }

    previous_back_state_ = joy->buttons[back_button_];
    previous_start_state_ = joy->buttons[start_button_];
  }
  else
  {
    // Ensure the trigger values have been initialized
    if ((joy->axes[brake_axes_] > parked_threshold_) && (joy->axes[throttle_axes_] > parked_threshold_))
    {
      initialized_ = true;
    }

    if (joy->axes[brake_axes_] <= parked_threshold_)
    {
      ROS_INFO("Pull the brake trigger to initialize.");
    }

    if (joy->axes[throttle_axes_] <= parked_threshold_)
    {
      ROS_INFO("Pull the throttle trigger to initilize.");
    }
  }
}

void RosccoTeleop::timerCallback(const ros::TimerEvent& event)
{
  if (enabled_)
  {
    roscco::BrakeCommand brake_msg;
    brake_msg.header.stamp = ros::Time::now();
    brake_msg.brake_position = brake_;
    brake_pub_.publish(brake_msg);

    roscco::ThrottleCommand throttle_msg;
    throttle_msg.header.stamp = ros::Time::now();
    throttle_msg.throttle_position = throttle_;
    throttle_pub_.publish(throttle_msg);

    // Utilize exponential average similar to OSCC's joystick commander for smoothing of joystick twitchy output
    steering_average_ = calc_exponential_average(steering_average_, steering_, data_smoothing_factor);

    roscco::SteeringCommand steering_msg;
    steering_msg.header.stamp = ros::Time::now();
    steering_msg.steering_torque = steering_average_;
    steering_pub_.publish(steering_msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "roscco_teleop");
  RosccoTeleop roscco_teleop;

  ros::spin();
}

// Calculate the exponential average
double calc_exponential_average(const double average, const double setpoint, const double factor)
{
  double exponential_average = (setpoint * factor) + ((1.0 - factor) * average);

  return (exponential_average);
}

// Repmap the value in an existing linear range to an new linear range example 0 in [-1, 1] to [0, 1] results in 0.5
double linear_tranformation(const double value, const double high_1, const double low_1, const double high_2, const double low_2)
{
  return low_2 + (value - low_1) * (high_2 - low_2) / (high_1 - low_1);
}
