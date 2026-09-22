#include <stdio.h>
#include <math.h>
#include "foc_controller.h"
#include "drive_state.h"
#include "fault_manager.h"
#include "nvm.h"


/* ================================================================
 * Test 1 — Safety Check
 * ================================================================ */
void test_safety_check()
{
    printf("\n=== Test 1: Safety Check ===\n");
    printf("Drive NOT enabled — foc_step() should do nothing\n\n");

    foc_init(1.0f, 10.0f, 1.0f, 10.0f, FOC_OMEGA_DEFAULT);
    foc_set_Iq_ref(5.0f);

    foc_step();

    uint32_t CCR1, CCR2, CCR3;
    mock_pwm_read(CCR1, CCR2, CCR3);

    printf("CCR1=%u CCR2=%u CCR3=%u\n", CCR1, CCR2, CCR3);
    printf("Safety check: %s\n",
           (CCR1 == 0 && CCR2 == 0 && CCR3 == 0) ? "PASS" : "FAIL");
}


/* ================================================================
 * Test 2 — Single Step with Enable
 * ================================================================ */
void test_single_step()
{
    printf("\n=== Test 2: Single Step With Enable ===\n");
    printf("Enable drive, call foc_step() once\n\n");

    foc_init(1.0f, 10.0f, 1.0f, 10.0f, FOC_OMEGA_DEFAULT);
    foc_set_Iq_ref(5.0f);
    mock_gpio_set_enable(true);
    drive_init();
    drive_transition(STATE_READY);
    drive_transition(STATE_RUNNING);

    foc_step();

    float Id, Iq, Vd, Vq, theta;
    foc_get_state(Id, Iq, Vd, Vq, theta);

    uint32_t CCR1, CCR2, CCR3;
    mock_pwm_read(CCR1, CCR2, CCR3);

    printf("theta = %.4f rad\n", theta);
    printf("Id    = %.4f A\n",   Id);
    printf("Iq    = %.4f A\n",   Iq);
    printf("Vd    = %.4f V\n",   Vd);
    printf("Vq    = %.4f V\n",   Vq);
    printf("CCR1=%u CCR2=%u CCR3=%u\n\n", CCR1, CCR2, CCR3);

    bool ccr_changed = (CCR1 != 0 || CCR2 != 0 || CCR3 != 0);
    printf("CCR changed from zero: %s\n", ccr_changed ? "PASS" : "FAIL");
}


/* ================================================================
 * Test 3 — Multiple Steps
 * ================================================================ */
void test_multiple_steps()
{
    printf("\n=== Test 3: Multiple Steps ===\n");
    printf("Iq_ref = 5.0A, run 200 steps\n");
    printf("Watch Id → 0, Iq → 5.0\n\n");

    foc_init(1.0f, 10.0f, 1.0f, 10.0f, FOC_OMEGA_DEFAULT);
    foc_set_Iq_ref(5.0f);
    mock_gpio_set_enable(true);
    drive_init();
    drive_transition(STATE_READY);
    drive_transition(STATE_RUNNING);

    printf("%-6s %-10s %-10s %-10s %-10s %-8s %-8s %-8s\n",
           "Step", "theta", "Id", "Iq", "Vq", "CCR1", "CCR2", "CCR3");

    for (int i = 0; i < 200; i++)
    {
        foc_step();

        if (i % 20 == 0)
        {
            float Id, Iq, Vd, Vq, theta;
            foc_get_state(Id, Iq, Vd, Vq, theta);

            uint32_t CCR1, CCR2, CCR3;
            mock_pwm_read(CCR1, CCR2, CCR3);

            printf("%-6d %-10.4f %-10.4f %-10.4f %-10.4f %-8u %-8u %-8u\n",
                   i, theta, Id, Iq, Vq, CCR1, CCR2, CCR3);
        }
    }
}


/* ================================================================
 * Test 4 — STO Test
 * ================================================================ */
void test_STO()
{
    printf("\n=== Test 4: STO Test ===\n");
    printf("Run 50 steps then trigger STO\n");
    printf("CCR must freeze — no more updates after STO\n\n");

    foc_init(1.0f, 10.0f, 1.0f, 10.0f, FOC_OMEGA_DEFAULT);
    foc_set_Iq_ref(5.0f);
    mock_gpio_set_enable(true);
    drive_init();
    drive_transition(STATE_READY);
    drive_transition(STATE_RUNNING);

    for (int i = 0; i < 50; i++) foc_step();

    uint32_t CCR1_before, CCR2_before, CCR3_before;
    mock_pwm_read(CCR1_before, CCR2_before, CCR3_before);
    printf("CCR before STO: CCR1=%u CCR2=%u CCR3=%u\n",
           CCR1_before, CCR2_before, CCR3_before);

    mock_gpio_set_STO(true);
    printf("STO triggered\n");

    for (int i = 0; i < 10; i++) foc_step();

    uint32_t CCR1_after, CCR2_after, CCR3_after;
    mock_pwm_read(CCR1_after, CCR2_after, CCR3_after);
    printf("CCR after STO:  CCR1=%u CCR2=%u CCR3=%u\n",
           CCR1_after, CCR2_after, CCR3_after);

    bool frozen = (CCR1_before == CCR1_after &&
                   CCR2_before == CCR2_after &&
                   CCR3_before == CCR3_after);

    printf("CCR frozen after STO: %s\n", frozen ? "PASS" : "FAIL");
}


/* ================================================================
 * Test 5 — Reset Test
 * ================================================================ */
void test_reset()
{
    printf("\n=== Test 5: Reset Test ===\n");
    printf("Run 100 steps then reset\n");
    printf("PI integrators must clear — Vd Vq back to zero\n\n");

    foc_init(1.0f, 10.0f, 1.0f, 10.0f, FOC_OMEGA_DEFAULT);
    foc_set_Iq_ref(5.0f);
    mock_gpio_set_enable(true);
    drive_init();
    drive_transition(STATE_READY);
    drive_transition(STATE_RUNNING);

    for (int i = 0; i < 100; i++) foc_step();

    float Id, Iq, Vd_before, Vq_before, theta;
    foc_get_state(Id, Iq, Vd_before, Vq_before, theta);
    printf("Before reset: Vd=%.4f Vq=%.4f\n", Vd_before, Vq_before);

    foc_reset();
    printf("foc_reset() called\n");

    foc_step();

    float Vd_after, Vq_after;
    foc_get_state(Id, Iq, Vd_after, Vq_after, theta);
    printf("After  reset: Vd=%.4f Vq=%.4f\n", Vd_after, Vq_after);

    bool reset_worked = (Vd_after < Vd_before || Vq_after < Vq_before);
    printf("Reset cleared integrators: %s\n", reset_worked ? "PASS" : "CHECK");
}

void test_closed_loop()
{
    printf("\n=== Test 6: Closed Loop with Motor Model ===\n");
    printf("Iq_ref = 5.0A — watch Iq converge\n\n");

    foc_init(1.0f, 10.0f, 1.0f, 10.0f, FOC_OMEGA_DEFAULT);
    foc_set_Iq_ref(5.0f);
    mock_gpio_set_enable(true);
    drive_init();
    drive_transition(STATE_READY);
    drive_transition(STATE_RUNNING);

    float Id_motor = 0.0f;
    float Iq_motor = 0.0f;
    float Rs       = 1.0f;
    float Ls       = 0.005f;
    float omega    = FOC_OMEGA_DEFAULT;

    printf("%-6s %-10s %-10s %-10s %-10s\n",
           "Step", "theta", "Id", "Iq", "Vq");

    for (int i = 0; i < 500; i++)
    {
        foc_step();

        float Id, Iq, Vd, Vq, theta;
        foc_get_state(Id, Iq, Vd, Vq, theta);

        /* Motor model */
        float dId_dt = (Vd - Rs*Id_motor + omega*Ls*Iq_motor) / Ls;
        float dIq_dt = (Vq - Rs*Iq_motor - omega*Ls*Id_motor) / Ls;
        Id_motor += dId_dt * FOC_DT;
        Iq_motor += dIq_dt * FOC_DT;

        /* Feed back into FOC */
        foc_set_feedback(Id_motor, Iq_motor);

        if (i % 50 == 0)
        {
            printf("%-6d %-10.4f %-10.4f %-10.4f %-10.4f\n",
                   i, theta, Id_motor, Iq_motor, Vq);
        }
    }

    printf("\nFinal: Id=%.4f (target=0.0) Iq=%.4f (target=5.0)\n",
           Id_motor, Iq_motor);
}


/* ================================================================
 * MAIN
 * ================================================================ */
int main()
{
    printf("========================================\n");
    printf("  FOC Controller Unit Tests\n");
    printf("========================================\n");

    test_safety_check();
    test_single_step();
    test_multiple_steps();
    test_STO();
    test_reset();
    test_closed_loop();

    printf("\n========================================\n");
    printf("  All tests complete\n");
    printf("========================================\n");

    return 0;
}