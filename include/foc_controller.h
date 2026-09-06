#ifndef FOC_CONTROLLER_H
#define FOC_CONTROLLER_H

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "pi_controller.h"
#include "foc_transforms.h"
#include "mock_hal.h"
#include "svpwm.h"

/**
 * FOC Controller
 * --------------
 * Orchestrates the complete Field Oriented Control loop.
 * Connects all modules — mock HAL, transforms, PI, SVPWM.
 *
 * One call to foc_step() performs one complete FOC cycle:
 *     Read encoder and currents
 *     Clarke and Park transforms
 *     PI control of Id and Iq
 *     Inverse Park
 *     SVPWM
 *     Write CCR values to mock PWM
 *
 * Designed to run every 100us in real firmware ISR.
 * In simulation — called from main loop.
 *
 * No dynamic memory. No exceptions. Static allocation only.
 */


/* ================================================================
 * CONSTANTS
 * ================================================================ */

#define FOC_DT              0.0001f     /* 100us — 10kHz FOC rate          */
#define FOC_VBUS            600.0f      /* DC bus voltage volts             */
#define FOC_ARR             8500U       /* TIM1 ARR — 10kHz at 170MHz       */
#define FOC_ID_REF          0.0f        /* Id reference — zero for PMSM     */
#define FOC_PEAK_CURRENT    10.0f       /* Peak phase current amps          */
#define FOC_OMEGA_DEFAULT   314.16f     /* Default speed — 50Hz motor       */


/* ================================================================
 * PUBLIC FUNCTIONS
 * ================================================================ */

/**
 * foc_init
 * --------
 * Initialise all FOC modules.
 * Call once at startup before any foc_step() calls.
 *
 * @param Kp_id       Id PI proportional gain
 * @param Ki_id       Id PI integral gain
 * @param Kp_iq       Iq PI proportional gain
 * @param Ki_iq       Iq PI integral gain
 * @param omega_start Starting motor speed rad/s
 */
void foc_init(float Kp_id, float Ki_id,
              float Kp_iq, float Ki_iq,
              float omega_start);


/**
 * foc_set_Iq_ref
 * --------------
 * Set torque current reference.
 * Called by operator interface or speed controller.
 *
 * @param Iq_ref  Torque current reference in amps
 */
void foc_set_Iq_ref(float Iq_ref);


/**
 * foc_step
 * --------
 * Execute one complete FOC cycle.
 * Call every 100us — equivalent to ISR in real firmware.
 *
 * Steps:
 *     0. Safety check — enable and STO
 *     1. Read encoder — theta, cos, sin
 *     2. Read currents — Ia, Ib
 *     3. Clarke transform
 *     4. Park transform
 *     5. PI controllers — Vd, Vq
 *     6. Inverse Park
 *     7. SVPWM
 *     8. Write CCR values
 *     9. Advance encoder
 *
 * Does nothing if enable is false or STO is active.
 */
void foc_step(void);


/**
 * foc_get_state
 * -------------
 * Read current FOC state for debugging and printing.
 *
 * @param Id     Last Id value amps
 * @param Iq     Last Iq value amps
 * @param Vd     Last Vd value volts
 * @param Vq     Last Vq value volts
 * @param theta  Last rotor angle radians
 */
void foc_get_state(float &Id, float &Iq,
                   float &Vd, float &Vq,
                   float &theta);


/**
 * foc_reset
 * ---------
 * Reset all PI controllers.
 * Call on fault clearance or drive stop.
 */
void foc_reset(void);


#endif /* FOC_CONTROLLER_H */
