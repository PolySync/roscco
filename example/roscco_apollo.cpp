#include "roscco_apollo.h"


RosccoApollo::RosccoApollo()
{
    brake_pub = nh.advertise<roscco::BrakeCommand>( "brake_command", 1 );
    throttle_pub = nh.advertise<roscco::ThrottleCommand>( "throttle_command", 1 );
    steering_pub = nh.advertise<roscco::SteeringCommand>( "steering_command", 1 );
    chassis_pub = nh.advertise<apollo::canbus::Chassis>( "/apollo/canbus/chassis", 1 );

    steering_sub = nh.subscribe( "/apollo/control", 1, &RosccoApollo::steeringCallback, this );  
    brake_sub = nh.subscribe( "/apollo/control", 1, &RosccoApollo::brakeCallback, this );
    throttle_sub = nh.subscribe( "/apollo/control", 1, &RosccoApollo::throttleCallback, this );
    localization_sub = nh.subscribe( "/apollo/localization/pose", 1, &RosccoApollo::localizationCallback, this );

    can_frame_sub = nh.subscribe( "/can_frame", 1, &RosccoApollo::EVCanFrameCallback, this );
}


void RosccoApollo::steeringCallback( const apollo::control::ControlCommand& input ) 
{
    roscco::SteeringCommand output;

    output.header.stamp = ros::Time::now();
    closedLoopControl( input.steering_target(), output, steering_angle_report );

    steering_pub.publish( output );
}


void RosccoApollo::brakeCallback( const apollo::control::ControlCommand& input ) 
{
    roscco::BrakeCommand output;

    output.header.stamp = ros::Time::now();
    output.brake_position = input.brake() / 100;

    brake_pub.publish( output );
}


void RosccoApollo::throttleCallback( const apollo::control::ControlCommand& input ) 
{
    roscco::ThrottleCommand output;

    output.header.stamp = ros::Time::now();
    output.throttle_position = input.throttle() / 100;

    throttle_pub.publish( output );
}


void RosccoApollo::EVCanFrameCallback( const roscco::CanFrame& input ) 
{
    switch( input.frame.can_id )
    {
        case 512: 
        {
            throttle_report = input.frame.data[4];

            throttle_report = throttle_report * THROTTLE_RATIO;

            break;
        }
        case 544: 
        {
            brake_report = input.frame.data[4] + input.frame.data[5] * 256;

            brake_report = brake_report * EV_BRAKE_RATIO;

            break;
        }
        case 688: 
        {
            steering_angle_report = input.frame.data[0] + input.frame.data[1] * 256;

            if( steering_angle_report > 60000 ) 
            {
                steering_angle_report -= 65535;
            }

            steering_angle_report = steering_angle_report * STEERING_RATIO;

            break;
        }
        case 1200:
        {
            speed_report = input.frame.data[0] + input.frame.data[2]
                            + input.frame.data[4] + input.frame.data[6];

            speed_report += ( input.frame.data[1] + input.frame.data[3]
                            + input.frame.data[5] + input.frame.data[7] ) * 256;

            speed_report = speed_report / 4;

            speed_report = speed_report * EV_SPEED_RATIO;

            break;
        }
        default :
        {
            /// Empty default to leave no untested cases
            break;
        }
    }

    apollo::canbus::Chassis output;

    output.set_steering_percentage( steering_angle_report );
    output.set_throttle_percentage( throttle_report );
    output.set_brake_percentage( brake_report );
    output.set_speed_mps( speed_report );

    chassis_pub.publish( output );
}


void RosccoApollo::localizationCallback( const apollo::localization::LocalizationEstimate& input ) 
{
    std::cout << std::to_string( input.pose().position().x() ) << ", "
              << std::to_string( input.pose().position().y() ) << "\n";
}


void closedLoopControl( double setpoint, 
                        roscco::SteeringCommand& output,
                        double steering_angle_report )
{
    pid_terms params;
    pid_state state;

    createPIDState( setpoint, &state );

    params.max = 1;
    params.min = -1;
    params.p_term = 0.016;
    params.i_term = 0.0001;
    params.d_term = 0.01;
    params.i_max = 5000;

    output.steering_torque = pidController( &params, &state, steering_angle_report );
}


int main( int argc, char** argv )
{
    ros::init( argc, argv, "roscco_apollo" );
    RosccoApollo roscco_apollo;

    ros::spin();
}
