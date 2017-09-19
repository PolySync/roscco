#include <geometry_msgs/Accel.h>
#include <ros/ros.h>
#include <roscco/BrakeCommand.h>
#include <roscco/EnableDisable.h>
#include <roscco/SteeringCommand.h>
#include <roscco/ThrottleCommand.h>
#include <sensor_msgs/Joy.h>

double calc_exponential_average(double average, double setpoint, double factor);

class TeleopRoscco
{
public:
  TeleopRoscco();

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

  int previous_start_state_;
  int previous_back_state_;

  // Store last known value for timed callback
  double brake_;
  double throttle_;
  double steering_;
  bool enabled_ = false;

  double steering_average_;

  // Variable to ensure joystick triggers have been initialized_
  bool initialized_ = false;

  int brake_axes_ = 2;
  int throttle_axes_ = 5;
  int steering_axes_ = 0;
  int start_button_ = 7;
  int back_button_ = 6;
};

TeleopRoscco::TeleopRoscco() : brake_(0.0), throttle_(0.0), steering_(0.0), initialized_(false)
{
  int queue_size = 10;

  brake_pub_ = nh_.advertise<roscco::BrakeCommand>("brake_command", queue_size);
  throttle_pub_ = nh_.advertise<roscco::ThrottleCommand>("throttle_command", queue_size);
  steering_pub_ = nh_.advertise<roscco::SteeringCommand>("steering_command", queue_size);
  enable_disable_pub_ = nh_.advertise<roscco::EnableDisable>("enable_disable", queue_size);

  // Timed callback for publishing to OSCC < 200 ms
  timer_ = nh_.createTimer(ros::Duration(0.05), &TeleopRoscco::timerCallback, this);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", queue_size, &TeleopRoscco::joystickCallback, this);
}

void TeleopRoscco::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // Validate the gamepad triggers are not
  if (initialized_)
  {
    // Map the joystick values [-1, 1] to oscc values [0, 1]
    brake_ = (joy->axes[brake_axes_] - 1) / (double)-2;
    throttle_ = (joy->axes[throttle_axes_] - 1) / (double)-2;

    // Joystick maps inversely to oscc steering
    steering_ = joy->axes[steering_axes_] * -1;

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
    // The threshold for considering the controller triggers to be parked in the correct position
    double parked_threshold_ = 0.99;

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

void TeleopRoscco::timerCallback(const ros::TimerEvent& event)
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

    // Smooth the steering to remove twitchy joystick movements
    double data_smoothing_factor = 0.1;
    steering_average_ = calc_exponential_average(steering_average_, steering_, data_smoothing_factor);

    roscco::SteeringCommand steering_msg;
    steering_msg.header.stamp = ros::Time::now();
    steering_msg.steering_torque = steering_average_;
    steering_pub_.publish(steering_msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopRoscco teleop_roscco;

  ros::spin();
}

double calc_exponential_average(double average, double setpoint, double factor)
{
  double exponential_average = (setpoint * factor) + ((1.0 - factor) * average);

  return (exponential_average);
}
