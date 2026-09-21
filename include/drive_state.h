#ifndef DRIVE_STATE_H
#define DRIVE_STATE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Drive State Machine
 * -------------------
 * Manages VFD drive states and transitions.
 * Enforces valid transitions — invalid ones rejected.
 *
 * States:
 *     INIT               — power on, self test, load parameters
 *     READY              — healthy, waiting for run command
 *     RUNNING            — motor spinning, FOC active
 *     ALARM              — warning condition, motor still running
 *     FAULT_RECOVERABLE  — fault, motor stopped, operator can reset
 *     FAULT_LOCKOUT      — critical fault, engineer must clear
 *
 * No dynamic memory. No exceptions. Static allocation only.
 */


/* ================================================================
 * DRIVE STATES
 * ================================================================ */

typedef enum
{
    STATE_INIT              = 0,
    STATE_READY             = 1,
    STATE_RUNNING           = 2,
    STATE_ALARM             = 3,
    STATE_FAULT_RECOVERABLE = 4,
    STATE_FAULT_LOCKOUT     = 5
} DriveState;


/* ================================================================
 * FAULT CODES
 * ================================================================ */

typedef enum
{
    FAULT_NONE               = 0,   /* no fault                          */
    FAULT_OVERCURRENT        = 1,   /* phase current exceeded limit      */
    FAULT_OVERVOLTAGE        = 2,   /* DC bus voltage too high           */
    FAULT_UNDERVOLTAGE       = 3,   /* DC bus voltage too low            */
    FAULT_OVERTEMPERATURE    = 4,   /* IGBT or motor temperature too high*/
    FAULT_ENCODER_LOSS       = 5,   /* encoder signal lost               */
    FAULT_IGBT_DESAT         = 6,   /* IGBT desaturation — short circuit */
    FAULT_POWER_LOSS         = 7,   /* mains power lost                  */
    FAULT_EARTH              = 8,   /* earth fault detected              */
    ALARM_TEMP_WARNING       = 9,   /* temperature rising — not yet trip */
    ALARM_CURRENT_HIGH       = 10,  /* current high — not yet trip       */
    ALARM_VBUS_HIGH          = 11   /* DC bus rising — not yet trip      */
} FaultCode;


/* ================================================================
 * FAULT SEVERITY
 * ================================================================ */

typedef enum
{
    SEVERITY_NONE        = 0,   /* normal operation                  */
    SEVERITY_ALARM       = 1,   /* warning — motor keeps running     */
    SEVERITY_RECOVERABLE = 2,   /* fault — operator can reset        */
    SEVERITY_LOCKOUT     = 3    /* critical — engineer must clear     */
} FaultSeverity;


/* ================================================================
 * FAULT ENTRY
 * Stored in circular buffer — one entry per fault event
 * ================================================================ */

typedef struct
{
    uint32_t   timestamp_ms;    /* milliseconds since power on       */
    FaultCode  code;            /* what faulted                      */
    DriveState state_at_fault;  /* drive state when fault occurred   */
    float      Vbus;            /* DC bus voltage at fault (V)       */
    float      Ia;              /* phase A current at fault (A)      */
    float      Id;              /* flux current at fault (A)         */
    float      Iq;              /* torque current at fault (A)       */
    float      speed_rpm;       /* motor speed at fault (RPM)        */
} FaultEntry;


/* ================================================================
 * DRIVE PARAMETERS
 * Saved to and loaded from NVM
 * ================================================================ */

typedef struct
{
    /* Motor parameters */
    float Rs;                   /* stator resistance ohms            */
    float Ls;                   /* stator inductance henries         */
    float rated_current;        /* rated phase current amps          */
    float rated_speed_rpm;      /* rated motor speed RPM             */

    /* PI gains */
    float Kp_id;                /* Id PI proportional gain           */
    float Ki_id;                /* Id PI integral gain               */
    float Kp_iq;                /* Iq PI proportional gain           */
    float Ki_iq;                /* Iq PI integral gain               */

    /* Speed limits */
    float speed_min_rpm;        /* minimum allowed speed RPM         */
    float speed_max_rpm;        /* maximum allowed speed RPM         */
    float accel_rate;           /* acceleration rate RPM per second  */
    float decel_rate;           /* deceleration rate RPM per second  */

    /* Protection thresholds */
    float overcurrent_limit;    /* overcurrent trip threshold amps   */
    float overvoltage_limit;    /* overvoltage trip threshold volts  */
    float undervoltage_limit;   /* undervoltage trip threshold volts */
    float overtemp_limit;       /* overtemperature trip threshold C  */
    float temp_alarm_limit;     /* temperature alarm threshold C     */
} DriveParams;


/* ================================================================
 * STATE MACHINE FUNCTIONS
 * ================================================================ */

/**
 * drive_init
 * ----------
 * Initialise state machine.
 * Sets state to INIT.
 * Loads parameters from NVM.
 * Runs self test.
 * Call once at power on.
 */
void drive_init(void);

/**
 * drive_get_state
 * ---------------
 * Returns current drive state.
 */
DriveState drive_get_state(void);

/**
 * drive_transition
 * ----------------
 * Request transition to new state.
 * Validates transition is allowed.
 * Returns true if transition succeeded.
 * Returns false if transition invalid.
 *
 * @param new_state  Requested new state
 * @return           true = transitioned, false = rejected
 */
bool drive_transition(DriveState new_state);

/**
 * drive_state_name
 * ----------------
 * Returns human readable state name string.
 * For printing and logging.
 *
 * @param state  State to name
 * @return       String name of state
 */
const char* drive_state_name(DriveState state);

/**
 * drive_fault_name
 * ----------------
 * Returns human readable fault code name.
 *
 * @param code  Fault code
 * @return      String name of fault
 */
const char* drive_fault_name(FaultCode code);

/**
 * drive_get_params
 * ----------------
 * Returns pointer to current drive parameters.
 */
const DriveParams* drive_get_params(void);

/**
 * drive_set_params
 * ----------------
 * Update drive parameters.
 * Saves to NVM automatically.
 *
 * @param params  New parameters to apply
 */
void drive_set_params(const DriveParams* params);

/**
 * drive_get_uptime_ms
 * -------------------
 * Returns milliseconds since power on.
 * Used for fault timestamps.
 */
uint32_t drive_get_uptime_ms(void);

/**
 * drive_tick_ms
 * -------------
 * Advance uptime counter by 1ms.
 * Call from 1ms timer interrupt or main loop.
 */
void drive_tick_ms(void);


#endif /* DRIVE_STATE_H */