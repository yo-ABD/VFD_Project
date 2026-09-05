#ifndef FOC_TRANSFORMS_H
#define FOC_TRANSFORMS_H

#include <math.h>

/**
 * FOC Transforms
 * --------------
 * Clarke  — 3 phase stationary  to 2 phase stationary (alpha/beta)
 * Park    — 2 phase stationary  to 2 phase rotating   (d/q)
 * Inv Park— 2 phase rotating    to 2 phase stationary (alpha/beta)
 *
 * cos_theta and sin_theta are passed in precomputed.
 * Caller computes once per ISR cycle — not repeated inside each function.
 * This saves CPU cycles in the 100us current loop.
 *
 * No internal state — pure math functions.
 * No init or reset needed.
 * No dynamic memory. No exceptions. float only.
 */


/* Compile time constant — never recalculated at runtime */
#define ONE_OVER_SQRT3  0.57735026919f   /* 1 / sqrt(3) */
#define TWO_OVER_SQRT3  1.15470053838f   /* 2 / sqrt(3) */


/**
 * clarke
 * ------
 * Converts 3 phase currents to 2 phase alpha/beta stationary frame.
 * Uses power-invariant Clarke transformation.
 * Ic is derived internally via Kirchhoff: Ic = -(Ia + Ib)
 * No third current sensor needed.
 *
 * Formula:
 *     Ialpha = Ia
 *     Ibeta  = (Ia + 2*Ib) / sqrt(3)
 *
 * @param Ia      Phase A current (amps)
 * @param Ib      Phase B current (amps)
 * @param Ialpha  Output — alpha axis current (amps)
 * @param Ibeta   Output — beta  axis current (amps)
 */
void clarke(float Ia, float Ib,
            float &Ialpha, float &Ibeta);


/**
 * park
 * ----
 * Converts alpha/beta stationary frame to d/q rotating frame.
 * Rotating frame turns with rotor — sine waves become DC values.
 *
 * Formula:
 *     Id =  Ialpha * cos(theta) + Ibeta * sin(theta)
 *     Iq = -Ialpha * sin(theta) + Ibeta * cos(theta)
 *
 * cos_theta and sin_theta must be precomputed by caller:
 *     float cos_theta = cosf(theta);
 *     float sin_theta = sinf(theta);
 *
 * @param Ialpha     Alpha axis current (amps)
 * @param Ibeta      Beta  axis current (amps)
 * @param cos_theta  Precomputed cosine of rotor angle
 * @param sin_theta  Precomputed sine   of rotor angle
 * @param Id         Output — flux     current (amps) — target = 0 for PMSM
 * @param Iq         Output — torque   current (amps) — target = speed demand
 */
void park(float Ialpha, float Ibeta,
          float cos_theta, float sin_theta,
          float &Id, float &Iq);


/**
 * inverse_park
 * ------------
 * Converts d/q rotating frame voltages back to alpha/beta stationary frame.
 * SVPWM operates in stationary frame — this prepares its inputs.
 *
 * Formula:
 *     Valpha = Vd * cos(theta) - Vq * sin(theta)
 *     Vbeta  = Vd * sin(theta) + Vq * cos(theta)
 *
 * Same cos_theta and sin_theta as Park — same theta, same ISR cycle.
 * No additional trigonometry cost.
 *
 * @param Vd         Flux     voltage demand from Id PI controller (volts)
 * @param Vq         Torque   voltage demand from Iq PI controller (volts)
 * @param cos_theta  Precomputed cosine of rotor angle
 * @param sin_theta  Precomputed sine   of rotor angle
 * @param Valpha     Output — alpha axis voltage for SVPWM (volts)
 * @param Vbeta      Output — beta  axis voltage for SVPWM (volts)
 */
void inverse_park(float Vd, float Vq,
                  float cos_theta, float sin_theta,
                  float &Valpha, float &Vbeta);


#endif /* FOC_TRANSFORMS_H */
