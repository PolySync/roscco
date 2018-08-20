#include <ros/ros.h>

typedef struct
{
  double max;
  double min;
  double p_term;
  double i_term;
  double d_term;
  double i_max;
} pid_terms;

typedef struct
{
  double integral;
  double pre_error;
  double setpoint;
} pid_state;

/**
 * @brief define a PID state
 * 
 * @param setpoint/target
 * @param PID state
 */
void createPIDState( double setpoint, pid_state *state );

/**
 * @brief PID controller
 * 
 * @param PID terms (max, min, p_term, i_term, d_term, i_max) 
 * @param PID state
 * @param measured process variable
 */
double pidController( pid_terms *terms, pid_state *state, double position );
