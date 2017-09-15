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
  bool enabled_;

  double steering_average;

  // Variable to ensure joystick triggers have been initialized_
  bool initialized_;
};

TeleopRoscco::TeleopRoscco() : brake_(0.0), throttle_(0.0), steering_(0.0), initialized_(false)
{
  brake_pub_ = nh_.advertise<roscco::BrakeCommand>("brake_command", 1);
  throttle_pub_ = nh_.advertise<roscco::ThrottleCommand>("throttle_command", 1);
  steering_pub_ = nh_.advertise<roscco::SteeringCommand>("steering_command", 1);
  enable_disable_pub_ = nh_.advertise<roscco::EnableDisable>("enable_disable", 1);

  // Timed callback for publishing to OSCC < 200 ms
  timer_ = nh_.createTimer(ros::Duration(0.05), &TeleopRoscco::timerCallback, this);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopRoscco::joystickCallback, this);
}

void TeleopRoscco::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if (initialized_)
  {
    brake_ = (joy->axes[2] - 1) / (double)-4;
    throttle_ = (joy->axes[5] - 1) / (double)-4;
    steering_ = joy->axes[0] * -0.5;

    roscco::EnableDisable enable_msg;
    enable_msg.header.stamp = ros::Time::now();

    if ((previous_back_state_ == 0) && joy->buttons[6])
    {
      enable_msg.enable_control = false;
      enable_disable_pub_.publish(enable_msg);
      enabled_ = false;
    }
    else if ((previous_start_state_ == 0) && joy->buttons[7] == 1)
    {
      enable_msg.enable_control = true;
      enable_disable_pub_.publish(enable_msg);
      enabled_ = true;
    }

    previous_back_state_ = joy->buttons[6];
    previous_start_state_ = joy->buttons[7];
  }
  else
  {
    // Ensure the trigger values have been initialized
    if ((joy->axes[2] > 0.99) && (joy->axes[5] > 0.99))
    {
      initialized_ = true;
    }

    if (joy->axes[2] <= 0.99)
    {
      ROS_INFO("Pull the brake trigger to initialize.");
    }

    if (joy->axes[5] <= 0.99)
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

    roscco::SteeringCommand steering_msg;
    steering_msg.header.stamp = ros::Time::now();
    steering_msg.steering_torque = calc_exponential_average(steering_average, steering_, 0.1);
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
