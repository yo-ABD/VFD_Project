#include "pi_controller.h"


/**
 * pi_init
 * -------
 * Sets tuning parameters and output limits.
 * Zeros all internal state.
 */
void pi_init(PIController &pi,
             float Kp,
             float Ki,
             float dt,
             float output_min,
             float output_max)
{
    pi.Kp         = Kp;
    pi.Ki         = Ki;
    pi.dt         = dt;
    pi.output_min = output_min;
    pi.output_max = output_max;

    /* Zero internal state */
    pi.integrator  = 0.0f;
    pi.prev_error  = 0.0f;
    pi.prev_output = 0.0f;
}


/**
 * pi_update
 * ---------
 * One PI iteration.
 *
 * Steps:
 *   1. P term  = Kp * error
 *   2. Integrate only if output was NOT saturated last cycle (anti-windup)
 *   3. I term  = integrator
 *   4. Sum output = P + I
 *   5. Clamp output to [output_min, output_max]
 *   6. Store output for next cycle anti-windup check
 */
float pi_update(PIController &pi, float error)
{
    /* Step 1 — Proportional term */
    float p_term = pi.Kp * error;

    /* Step 2 — Anti-windup: only integrate if previous output was not saturated */
    bool saturated = (pi.prev_output >= pi.output_max) ||
                     (pi.prev_output <= pi.output_min);

    if (!saturated)
    {
        pi.integrator += pi.Ki * error * pi.dt;
    }

    /* Step 3 — Integral term */
    float i_term = pi.integrator;

    /* Step 4 — Sum */
    float output = p_term + i_term;

    /* Step 5 — Clamp output */
    if (output > pi.output_max)
    {
        output = pi.output_max;
    }
    else if (output < pi.output_min)
    {
        output = pi.output_min;
    }

    /* Step 6 — Store for next cycle */
    pi.prev_error  = error;
    pi.prev_output = output;

    return output;
}


/**
 * pi_reset
 * --------
 * Zeros internal state.
 * Does NOT touch tuning parameters or limits.
 */
void pi_reset(PIController &pi)
{
    pi.integrator  = 0.0f;
    pi.prev_error  = 0.0f;
    pi.prev_output = 0.0f;
}

