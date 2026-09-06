#ifndef SVPWM_H
#define SVPWM_H

#include <stdint.h>
#include <math.h>

/**
 * SVPWM — Space Vector Pulse Width Modulation
 * --------------------------------------------
 * Converts voltage vector (Valpha, Vbeta) from inverse Park transform
 * into three PWM compare register values (CCR1, CCR2, CCR3) for TIM1.
 *
 * Why SVPWM over SPWM:
 *     SPWM  — 78.5% DC bus utilisation
 *     SVPWM — 90.7% DC bus utilisation
 *     15% more output voltage from same DC bus
 *     Lower harmonic distortion
 *     Industry standard for FOC motor drives
 *
 * How it works:
 *     3 phase inverter has 8 possible switching states
 *     6 active vectors + 2 zero vectors
 *     These 6 active vectors divide space into 6 sectors (60 deg each)
 *     Desired voltage vector approximated by combining:
 *         Two adjacent active vectors (T1 and T2 time)
 *         Zero vectors for remaining time (T0)
 *     T1 T2 T0 converted to phase on-times Ta Tb Tc
 *     Phase on-times normalised to CCR values
 *
 * No internal state — pure math.
 * No dynamic memory. No exceptions. float and uint32_t only.
 */


/* ================================================================
 * CONSTANTS
 * ================================================================ */

#define SVPWM_SQRT3         1.73205080757f      /* sqrt(3)       */
#define SVPWM_SQRT3_OVER_2  0.86602540378f      /* sqrt(3) / 2   */
#define SVPWM_ONE_OVER_SQRT3 0.57735026919f     /* 1 / sqrt(3)   */
#define SVPWM_TWO_OVER_SQRT3 1.15470053838f     /* 2 / sqrt(3)   */


/* ================================================================
 * PUBLIC FUNCTION
 * ================================================================ */

/**
 * svpwm
 * -----
 * Main SVPWM calculation. Call once per FOC cycle.
 * Takes voltage vector from inverse Park, outputs CCR values for TIM1.
 *
 * Steps internally:
 *     1. Normalise Valpha Vbeta against Vbus
 *     2. Detect sector (1 to 6)
 *     3. Calculate T1, T2, T0 for that sector
 *     4. Calculate phase on-times Ta, Tb, Tc
 *     5. Convert to CCR1, CCR2, CCR3
 *
 * @param Valpha  Alpha axis voltage from inverse Park (volts)
 * @param Vbeta   Beta  axis voltage from inverse Park (volts)
 * @param Vbus    DC bus voltage (volts) — typically 600V for 400V drive
 * @param ARR     TIM1 auto reload register — sets PWM period
 *                CCR values range from 0 to ARR
 * @param CCR1    Output — Phase A compare register value
 * @param CCR2    Output — Phase B compare register value
 * @param CCR3    Output — Phase C compare register value
 */
void svpwm(float    Valpha,
           float    Vbeta,
           float    Vbus,
           uint32_t ARR,
           uint32_t &CCR1,
           uint32_t &CCR2,
           uint32_t &CCR3);


/* ================================================================
 * INTERNAL HELPERS — declared here for unit testing
 * In production these would be static in .cpp
 * Exposed here so test file can call them directly
 * ================================================================ */

/**
 * svpwm_get_sector
 * ----------------
 * Determines which of the 6 sectors the voltage vector is in.
 * Uses sign of three reference voltages to determine sector.
 *
 * @param Valpha  Alpha axis voltage
 * @param Vbeta   Beta  axis voltage
 * @return        Sector number 1 to 6
 */
int svpwm_get_sector(float Valpha, float Vbeta);


/**
 * svpwm_calc_XYZ
 * --------------
 * Calculates normalised reference voltages X, Y, Z.
 * These are intermediate values used for T1 T2 calculation.
 * Normalised against Vbus so result is dimensionless ratio.
 *
 * @param Valpha  Alpha axis voltage
 * @param Vbeta   Beta  axis voltage
 * @param Vbus    DC bus voltage
 * @param X       Output — normalised reference voltage X
 * @param Y       Output — normalised reference voltage Y
 * @param Z       Output — normalised reference voltage Z
 */
void svpwm_calc_XYZ(float Valpha, float Vbeta, float Vbus,
                    float &X, float &Y, float &Z);


/**
 * svpwm_calc_T1_T2
 * ----------------
 * Calculates active vector times T1 and T2 for given sector.
 * T1 = time spent on first active vector of the sector
 * T2 = time spent on second active vector of the sector
 * T0 = Ts - T1 - T2 = time spent on zero vectors
 *
 * T1 + T2 clamped to Ts — prevents overmodulation.
 *
 * @param sector  Sector number 1 to 6
 * @param X       Normalised reference voltage X
 * @param Y       Normalised reference voltage Y
 * @param Z       Normalised reference voltage Z
 * @param Ts      Switching period in seconds (e.g. 0.0001 for 10kHz)
 * @param T1      Output — first  active vector time (seconds)
 * @param T2      Output — second active vector time (seconds)
 */
void svpwm_calc_T1_T2(int sector, float X, float Y, float Z,
                      float Ts, float &T1, float &T2);


/**
 * svpwm_calc_CCR
 * --------------
 * Converts T1 T2 T0 into CCR1 CCR2 CCR3 for given sector.
 * Phase on-times Ta Tb Tc calculated from T1 T2 T0.
 * Normalised to ARR counts.
 *
 * @param sector  Sector number 1 to 6
 * @param T1      First  active vector time (seconds)
 * @param T2      Second active vector time (seconds)
 * @param T0      Zero   vector time (seconds)
 * @param Ts      Switching period (seconds)
 * @param ARR     Timer auto reload value
 * @param CCR1    Output — Phase A compare value
 * @param CCR2    Output — Phase B compare value
 * @param CCR3    Output — Phase C compare value
 */
void svpwm_calc_CCR(int sector, float T1, float T2, float T0,
                    float Ts, uint32_t ARR,
                    uint32_t &CCR1, uint32_t &CCR2, uint32_t &CCR3);


#endif /* SVPWM_H */