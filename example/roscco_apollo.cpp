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

    can_frame_sub = nh.subscribe( "/can_frame", 1, &RosccoApollo::canFrameCallback, this );
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


void RosccoApollo::canFrameCallback( const roscco::CanFrame& input ) 
{
    switch( input.frame.can_id )
    {
        case KIA_SOUL_OBD_THROTTLE_PRESSURE_CAN_ID: 
        {
            #if defined( KIA_SOUL_EV )
                throttle_report = input.frame.data[4];
            #elif defined( KIA_NIRO )
                throttle_report = input.frame.data[7];
            #endif

            throttle_report = throttle_report * THROTTLE_RATIO;

            break;
        }
        case KIA_SOUL_OBD_BRAKE_PRESSURE_CAN_ID: 
        {
            #if defined( KIA_SOUL_EV )
                brake_report = input.frame.data[4] + input.frame.data[5] * 256;
            #elif defined( KIA_NIRO )
                brake_report = input.frame.data[3] + input.frame.data[4] * 256;
            #endif

            brake_report = brake_report * BRAKE_RATIO;

            break;
        }
        case KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID: 
        {
            steering_angle_report = input.frame.data[0] + input.frame.data[1] * 256;

            if( steering_angle_report > 60000 ) 
            {
                steering_angle_report -= 65535;
            }

            steering_angle_report = steering_angle_report * STEERING_RATIO;

            break;
        }
        case KIA_SOUL_OBD_SPEED_CAN_ID:
        {
            #if defined( KIA_SOUL_EV )
                speed_report = input.frame.data[3] + input.frame.data[2] * 128;

                speed_report = speed_report * SPEED_RATIO;

            #elif defined( KIA_NIRO )
                speed_report = input.frame.data[0];

                speed_report = speed_report * SPEED_RATIO;
            
            #endif

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
    params.i_term = 0.004;
    params.d_term = 0.001;
    params.i_max = 250;

    output.steering_torque = pidController( &params, &state, steering_angle_report );
}


int main( int argc, char** argv )
{
    ros::init( argc, argv, "roscco_apollo" );
    RosccoApollo roscco_apollo;

    ros::spin();
}
