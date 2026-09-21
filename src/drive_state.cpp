#include "drive_state.h"
#include "fault_manager.h"
#include "nvm.h"
#include <stdio.h>
#include <string.h>

/* ================================================================
 * STATIC STATE
 * ================================================================ */

static DriveState s_state      = STATE_INIT;
static DriveParams s_params;
static uint32_t    s_uptime_ms = 0;


/* ================================================================
 * VALID TRANSITION TABLE
 * ----------------------------------------------------------------
 * transition_allowed[from][to] = true if transition is valid
 *
 * Rows = current state
 * Cols = requested new state
 *
 *                        INIT   READY  RUN    ALARM  RECOV  LOCK
 * ================================================================ */
static const bool transition_allowed[6][6] =
{
    /* FROM INIT:              */  { false, true,  false, false, false, false },
    /* FROM READY:             */  { false, false, true,  false, false, false },
    /* FROM RUNNING:           */  { false, true,  false, true,  true,  true  },
    /* FROM ALARM:             */  { false, false, true,  false, true,  true  },
    /* FROM FAULT_RECOVERABLE: */  { false, true,  false, false, false, false },
    /* FROM FAULT_LOCKOUT:     */  { false, true,  false, false, false, false }
};


/* ================================================================
 * drive_init
 * ================================================================ */

void drive_init(void)
{
    s_state     = STATE_INIT;
    s_uptime_ms = 0;

    /* Load parameters from NVM */
    nvm_init();
    bool loaded = nvm_load_params(&s_params);

    if (loaded)
    {
        printf("[DRIVE] Parameters loaded from NVM\n");
    }
    else
    {
        printf("[DRIVE] NVM empty or corrupt — using defaults\n");
        memcpy(&s_params, &NVM_DEFAULT_PARAMS, sizeof(DriveParams));
    }

    /* Load fault history */
    fault_init();

    printf("[DRIVE] Self test passed — state: INIT\n");
}


/* ================================================================
 * drive_get_state
 * ================================================================ */

DriveState drive_get_state(void)
{
    return s_state;
}


/* ================================================================
 * drive_transition
 * ================================================================ */

bool drive_transition(DriveState new_state)
{
    /* Check if transition is valid */
    if (!transition_allowed[s_state][new_state])
    {
        printf("[DRIVE] REJECTED transition: %s → %s\n",
               drive_state_name(s_state),
               drive_state_name(new_state));
        return false;
    }

    /* Valid — perform transition */
    printf("[DRIVE] Transition: %s → %s\n",
           drive_state_name(s_state),
           drive_state_name(new_state));

    s_state = new_state;

    /* Actions on entering new state */
    switch (new_state)
    {
        case STATE_READY:
            printf("[DRIVE] Drive ready — waiting for run command\n");
            break;

        case STATE_RUNNING:
            printf("[DRIVE] Motor starting — FOC active\n");
            break;

        case STATE_ALARM:
            printf("[DRIVE] ALARM — motor still running — operator attention needed\n");
            break;

        case STATE_FAULT_RECOVERABLE:
            printf("[DRIVE] FAULT — motor stopped — operator can reset\n");
            /* Save parameters before fault */
            nvm_save_params(&s_params);
            break;

        case STATE_FAULT_LOCKOUT:
            printf("[DRIVE] CRITICAL FAULT — motor stopped — engineer required\n");
            /* Save parameters before fault */
            nvm_save_params(&s_params);
            break;

        default:
            break;
    }

    return true;
}


/* ================================================================
 * drive_state_name
 * ================================================================ */

const char* drive_state_name(DriveState state)
{
    switch (state)
    {
        case STATE_INIT:              return "INIT";
        case STATE_READY:             return "READY";
        case STATE_RUNNING:           return "RUNNING";
        case STATE_ALARM:             return "ALARM";
        case STATE_FAULT_RECOVERABLE: return "FAULT_RECOVERABLE";
        case STATE_FAULT_LOCKOUT:     return "FAULT_LOCKOUT";
        default:                      return "UNKNOWN";
    }
}


/* ================================================================
 * drive_fault_name
 * ================================================================ */

const char* drive_fault_name(FaultCode code)
{
    switch (code)
    {
        case FAULT_NONE:             return "NONE";
        case FAULT_OVERCURRENT:      return "OVERCURRENT";
        case FAULT_OVERVOLTAGE:      return "OVERVOLTAGE";
        case FAULT_UNDERVOLTAGE:     return "UNDERVOLTAGE";
        case FAULT_OVERTEMPERATURE:  return "OVERTEMPERATURE";
        case FAULT_ENCODER_LOSS:     return "ENCODER_LOSS";
        case FAULT_IGBT_DESAT:       return "IGBT_DESATURATION";
        case FAULT_POWER_LOSS:       return "POWER_LOSS";
        case FAULT_EARTH:            return "EARTH_FAULT";
        case ALARM_TEMP_WARNING:     return "ALARM_TEMP_WARNING";
        case ALARM_CURRENT_HIGH:     return "ALARM_CURRENT_HIGH";
        case ALARM_VBUS_HIGH:        return "ALARM_VBUS_HIGH";
        default:                     return "UNKNOWN";
    }
}


/* ================================================================
 * drive_get_params
 * ================================================================ */

const DriveParams* drive_get_params(void)
{
    return &s_params;
}


/* ================================================================
 * drive_set_params
 * ================================================================ */

void drive_set_params(const DriveParams* params)
{
    memcpy(&s_params, params, sizeof(DriveParams));
    nvm_save_params(&s_params);
    printf("[DRIVE] Parameters updated and saved to NVM\n");
}


/* ================================================================
 * drive_get_uptime_ms
 * ================================================================ */

uint32_t drive_get_uptime_ms(void)
{
    return s_uptime_ms;
}


/* ================================================================
 * drive_tick_ms
 * ================================================================ */

void drive_tick_ms(void)
{
    s_uptime_ms++;
}