#include <stdio.h>
#include <string.h>
#include "drive_state.h"
#include "fault_manager.h"
#include "nvm.h"

/* ================================================================
 * Test 1 — Valid Transitions
 * --------------------------
 * Test every valid transition succeeds
 * ================================================================ */
void test_valid_transitions()
{
    printf("\n=== Test 1: Valid Transitions ===\n\n");

    drive_init();

    /* INIT → READY */
    bool ok = drive_transition(STATE_READY);
    printf("INIT → READY:    %s\n", ok ? "PASS" : "FAIL");

    /* READY → RUNNING */
    ok = drive_transition(STATE_RUNNING);
    printf("READY → RUNNING: %s\n", ok ? "PASS" : "FAIL");

    /* RUNNING → ALARM */
    ok = drive_transition(STATE_ALARM);
    printf("RUNNING → ALARM: %s\n", ok ? "PASS" : "FAIL");

    /* ALARM → RUNNING (condition cleared) */
    ok = drive_transition(STATE_RUNNING);
    printf("ALARM → RUNNING: %s\n", ok ? "PASS" : "FAIL");

    /* RUNNING → FAULT_RECOVERABLE */
    ok = drive_transition(STATE_FAULT_RECOVERABLE);
    printf("RUNNING → FAULT_RECOVERABLE: %s\n", ok ? "PASS" : "FAIL");

    /* FAULT_RECOVERABLE → READY (operator reset) */
    ok = drive_transition(STATE_READY);
    printf("FAULT_RECOVERABLE → READY:   %s\n", ok ? "PASS" : "FAIL");

    /* READY → RUNNING again */
    ok = drive_transition(STATE_RUNNING);
    printf("READY → RUNNING: %s\n", ok ? "PASS" : "FAIL");

    /* RUNNING → FAULT_LOCKOUT */
    ok = drive_transition(STATE_FAULT_LOCKOUT);
    printf("RUNNING → FAULT_LOCKOUT: %s\n", ok ? "PASS" : "FAIL");

    /* FAULT_LOCKOUT → READY (engineer clears) */
    ok = drive_transition(STATE_READY);
    printf("FAULT_LOCKOUT → READY:   %s\n", ok ? "PASS" : "FAIL");
}


/* ================================================================
 * Test 2 — Invalid Transitions
 * ----------------------------
 * Test every invalid transition is rejected
 * ================================================================ */
void test_invalid_transitions()
{
    printf("\n=== Test 2: Invalid Transitions ===\n\n");

    drive_init();
    drive_transition(STATE_READY);
    drive_transition(STATE_RUNNING);

    /* RUNNING → INIT — invalid */
    bool ok = drive_transition(STATE_INIT);
    printf("RUNNING → INIT rejected:              %s\n", !ok ? "PASS" : "FAIL");

    /* Back to FAULT_LOCKOUT */
    drive_transition(STATE_FAULT_LOCKOUT);

    /* FAULT_LOCKOUT → RUNNING — invalid, must go via READY */
    ok = drive_transition(STATE_RUNNING);
    printf("FAULT_LOCKOUT → RUNNING rejected:     %s\n", !ok ? "PASS" : "FAIL");

    /* FAULT_LOCKOUT → FAULT_RECOVERABLE — invalid */
    ok = drive_transition(STATE_FAULT_RECOVERABLE);
    printf("FAULT_LOCKOUT → FAULT_RECOV rejected: %s\n", !ok ? "PASS" : "FAIL");

    /* Reset to READY */
    drive_transition(STATE_READY);

    /* READY → FAULT_LOCKOUT — invalid, cannot fault without running */
    ok = drive_transition(STATE_FAULT_LOCKOUT);
    printf("READY → FAULT_LOCKOUT rejected:       %s\n", !ok ? "PASS" : "FAIL");

    /* READY → ALARM — invalid */
    ok = drive_transition(STATE_ALARM);
    printf("READY → ALARM rejected:               %s\n", !ok ? "PASS" : "FAIL");
}


/* ================================================================
 * Test 3 — Fault Logging and Circular Buffer
 * -------------------------------------------
 * Log 55 faults into buffer of 50
 * Verify oldest overwritten
 * Verify newest always at index 0
 * ================================================================ */
void test_fault_logging()
{
    printf("\n=== Test 3: Fault Logging and Circular Buffer ===\n\n");

    drive_init();
    drive_transition(STATE_READY);
    drive_transition(STATE_RUNNING);

    /* Log 55 faults — buffer holds 50 */
    printf("Logging 55 faults into buffer of 50...\n");

    for (int i = 0; i < 55; i++)
    {
        drive_tick_ms();
        drive_tick_ms();
        drive_tick_ms();

        /* Alternate between fault types */
        FaultCode code = (i % 2 == 0) ? FAULT_OVERCURRENT : ALARM_TEMP_WARNING;
        fault_log(code, 600.0f + i, 10.0f + i * 0.1f, 0.5f, 4.8f, 1450.0f);
    }

    printf("\nBuffer count: %u (expect 50)\n", fault_get_count());
    printf("Count check: %s\n\n", fault_get_count() == 50 ? "PASS" : "FAIL");

    /* Most recent fault — index 0 — should be fault 54 */
    FaultEntry newest = fault_get(0);
    printf("Newest fault (index 0):\n");
    fault_print_entry(&newest);

    /* Oldest fault — index 49 — should be fault 5 (55-50=5) */
    FaultEntry oldest = fault_get(49);
    printf("Oldest fault (index 49):\n");
    fault_print_entry(&oldest);

    /* Vbus of newest should be 600+54=654 */
    printf("\nNewest Vbus = %.1f (expect 654.0): %s\n",
           newest.Vbus,
           (newest.Vbus > 653.0f && newest.Vbus < 655.0f) ? "PASS" : "FAIL");
}


/* ================================================================
 * Test 4 — Full Scenario
 * ----------------------
 * Power on → running → alarm → fault → reset → running → lockout
 * ================================================================ */
void test_full_scenario()
{
    printf("\n=== Test 4: Full Drive Scenario ===\n\n");

    /* Power on */
    drive_init();
    printf("State: %s\n\n", drive_state_name(drive_get_state()));

    /* Self test passed — move to READY */
    drive_transition(STATE_READY);
    printf("State: %s\n\n", drive_state_name(drive_get_state()));

    /* Operator presses RUN */
    drive_transition(STATE_RUNNING);
    printf("State: %s\n\n", drive_state_name(drive_get_state()));

    /* Temperature rising — alarm */
    drive_tick_ms();
    fault_log(ALARM_TEMP_WARNING, 600.0f, 9.5f, 0.1f, 4.9f, 1500.0f);
    drive_transition(STATE_ALARM);
    printf("State: %s\n\n", drive_state_name(drive_get_state()));

    /* Temperature drops — back to running */
    drive_transition(STATE_RUNNING);
    printf("State: %s\n\n", drive_state_name(drive_get_state()));

    /* Overcurrent — recoverable fault */
    drive_tick_ms();
    fault_log(FAULT_OVERCURRENT, 610.0f, 16.5f, 0.3f, 5.2f, 1490.0f);
    drive_transition(STATE_FAULT_RECOVERABLE);
    printf("State: %s\n\n", drive_state_name(drive_get_state()));

    /* Operator resets */
    drive_transition(STATE_READY);
    drive_transition(STATE_RUNNING);
    printf("State after reset: %s\n\n", drive_state_name(drive_get_state()));

    /* IGBT desaturation — lockout */
    drive_tick_ms();
    fault_log(FAULT_IGBT_DESAT, 580.0f, 25.0f, 1.5f, 8.0f, 1200.0f);
    drive_transition(STATE_FAULT_LOCKOUT);
    printf("State: %s\n\n", drive_state_name(drive_get_state()));

    /* Operator tries to reset — must fail */
    printf("Operator tries reset (must fail):\n");
    bool ok = drive_transition(STATE_RUNNING);
    printf("Reset to RUNNING rejected: %s\n\n", !ok ? "PASS" : "FAIL");

    /* Engineer clears and goes to READY */
    drive_transition(STATE_READY);
    printf("After engineer clears: %s\n\n", drive_state_name(drive_get_state()));

    /* Print fault history */
    fault_print_all();
}


/* ================================================================
 * Test 5 — NVM Save and Load
 * --------------------------
 * Save parameters to NVM
 * Change them in RAM
 * Load from NVM
 * Verify original values restored
 * ================================================================ */
void test_nvm()
{
    printf("\n=== Test 5: NVM Save and Load ===\n\n");

    drive_init();

    /* Get default params */
    DriveParams params;
    memcpy(&params, drive_get_params(), sizeof(DriveParams));

    /* Modify some values */
    params.Kp_iq          = 2.5f;
    params.speed_max_rpm  = 1200.0f;
    params.overcurrent_limit = 12.0f;

    /* Save to NVM */
    drive_set_params(&params);
    printf("Saved: Kp_iq=%.1f speed_max=%.0f overcurrent=%.1f\n",
           params.Kp_iq, params.speed_max_rpm, params.overcurrent_limit);

    /* Simulate power cycle — reinit */
    drive_init();

    /* Load should restore saved values */
    const DriveParams* loaded = drive_get_params();
    printf("Loaded: Kp_iq=%.1f speed_max=%.0f overcurrent=%.1f\n",
           loaded->Kp_iq, loaded->speed_max_rpm, loaded->overcurrent_limit);

    bool pass = (loaded->Kp_iq == 2.5f &&
                 loaded->speed_max_rpm == 1200.0f &&
                 loaded->overcurrent_limit == 12.0f);

    printf("NVM save and load: %s\n", pass ? "PASS" : "FAIL");

    /* Print NVM status */
    nvm_print_status();
}


/* ================================================================
 * MAIN
 * ================================================================ */
int main()
{
    printf("========================================\n");
    printf("  Drive State Machine Unit Tests\n");
    printf("========================================\n");

    test_valid_transitions();
    test_invalid_transitions();
    test_fault_logging();
    test_full_scenario();
    test_nvm();

    printf("\n========================================\n");
    printf("  All tests complete\n");
    printf("========================================\n");

    return 0;
}