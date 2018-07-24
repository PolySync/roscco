#include "roscco/pid_control.h"

void createPIDState( double setpoint, pid_state *state )
{
    state->integral = 0;
    state->pre_error = 0;
    state->setpoint = setpoint;
}

double pidController( pid_terms *terms, pid_state *state, double position )
{
    ///Calculate error
    double error = position - state->setpoint;

    ///Proportional term
    double Pout = terms->p_term*error;

    ///Integral term
    state->integral += error;

    if( state->integral > terms->i_max )
    {
        state->integral = terms->i_max;
    }
    if( state->integral < -terms->i_max )
    {
        state->integral = -terms->i_max;
    }
  
    double Iout = terms->i_term*state->integral;

    ///Derivative term
    double derivative = error - state->pre_error;
    double Dout = terms->d_term*derivative;

    ///Calculate total output
    double output = Pout + Iout + Dout;

    ///Restrict to max/min
    if( output > terms->max )
    {
        output = terms->max;
    }
    else
    {
        if( output < terms->min )
        {
            output = terms->min;
        }
    }

    ///save error to previous error
    state->pre_error = error;

    return output;
}
