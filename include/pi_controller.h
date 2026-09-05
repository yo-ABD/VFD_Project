#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

/**
 * PI Controller
 * -------------
 * Used for Id and Iq current control loops in FOC.
 * No dynamic memory. Static allocation only.
 * No exceptions. Embedded safe.
 */

struct PIController
{
    /* --- Tuning parameters --- */
    float Kp;           /* Proportional gain                          */
    float Ki;           /* Integral gain                              */
    float dt;           /* Time step in seconds (e.g. 0.0001 = 100us)*/

    /* --- Output limits --- */
    float output_min;   /* Minimum allowed output value               */
    float output_max;   /* Maximum allowed output value               */

    /* --- Internal state --- */
    float integrator;   /* Accumulated integral term                  */
    float prev_error;   /* Previous error — reserved for future use   */
    float prev_output;  /* Previous output — used for anti-windup     */
};


/**
 * pi_init
 * -------
 * Initialises all fields of the PI controller.
 * Zeros internal state (integrator, prev_error, prev_output).
 * Call once at startup before any pi_update calls.
 *
 * @param pi         Reference to PIController struct
 * @param Kp         Proportional gain
 * @param Ki         Integral gain
 * @param dt         Time step in seconds
 * @param output_min Minimum clamped output
 * @param output_max Maximum clamped output
 */
void pi_init(PIController &pi,
             float Kp,
             float Ki,
             float dt,
             float output_min,
             float output_max);


/**
 * pi_update
 * ---------
 * Runs one PI iteration. Call every control loop cycle.
 * Implements clamped output and anti-windup.
 * Anti-windup: stops integrating when output is saturated.
 *
 * @param pi    Reference to PIController struct
 * @param error Current error (reference - feedback)
 * @return      Clamped controller output
 */
float pi_update(PIController &pi, float error);


/**
 * pi_reset
 * --------
 * Zeros integrator and prev_error.
 * Call on drive state transitions (e.g. READY->RUNNING, after fault).
 * Prevents stale integrator value causing jerk on restart.
 *
 * @param pi Reference to PIController struct
 */
void pi_reset(PIController &pi);


#endif /* PI_CONTROLLER_H */

