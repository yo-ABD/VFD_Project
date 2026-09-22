#include <stdio.h>
#include <math.h>
#include "foc_controller.h"

/**
 * main_sim.cpp
 * ------------
 * Full FOC simulation with closed loop motor model.
 *
 * Motor model equations (simplified PMSM):
 *     dId/dt = (Vd - Rs*Id + omega*Ls*Iq) / Ls
 *     dIq/dt = (Vq - Rs*Iq - omega*Ls*Id) / Ls
 *
 * Loop:
 *     foc_step()                    run FOC cycle
 *     foc_get_state()               read Vd Vq
 *     motor model                   calculate new Id Iq
 *     foc_set_feedback(Id, Iq)      feed back into PI
 *     print every 50 steps
 */


/* ================================================================
 * MOTOR PARAMETERS
 * ================================================================ */
static const float MOTOR_RS    = 1.0f;      /* stator resistance ohms    */
static const float MOTOR_LS    = 0.005f;    /* stator inductance henries */
static const float MOTOR_OMEGA = 314.16f;   /* 50Hz motor rad/s          */


/* ================================================================
 * SIMULATION PARAMETERS
 * ================================================================ */
static const int   SIM_STEPS   = 1000;      /* total steps to run        */
static const int   PRINT_EVERY = 50;        /* print every N steps       */
static const float IQ_REF      = 5.0f;      /* torque current reference  */


/* ================================================================
 * MOTOR MODEL
 * ------------
 * Takes Vd Vq from PI controllers
 * Returns new Id Iq based on motor physics
 * ================================================================ */
void motor_model(float Vd, float Vq,
                 float &Id, float &Iq)
{
    /* Rate of change of currents */
    float dId_dt = (Vd - MOTOR_RS * Id + MOTOR_OMEGA * MOTOR_LS * Iq)
                   / MOTOR_LS;

    float dIq_dt = (Vq - MOTOR_RS * Iq - MOTOR_OMEGA * MOTOR_LS * Id)
                   / MOTOR_LS;

    /* Euler integration — one time step */
    Id += dId_dt * FOC_DT;
    Iq += dIq_dt * FOC_DT;
}


/* ================================================================
 * PRINT HEADER
 * ================================================================ */
void print_header()
{
    printf("\n%-6s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s\n",
           "Step", "theta", "Id", "Iq",
           "Id_err", "Iq_err", "Vd", "Vq", "CCR1");
    printf("%-6s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s\n",
           "----", "-----", "--", "--",
           "------", "------", "--", "--", "----");
}


/* ================================================================
 * PRINT STATE
 * ================================================================ */
void print_state(int step, float Id, float Iq,
                 float Vd, float Vq, float theta)
{
    uint32_t CCR1, CCR2, CCR3;
    mock_pwm_read(CCR1, CCR2, CCR3);

    float Id_err = FOC_ID_REF - Id;
    float Iq_err = IQ_REF     - Iq;

    printf("%-6d %-8.3f %-8.3f %-8.3f %-8.3f %-8.3f %-8.3f %-8.3f %-8u\n",
           step, theta, Id, Iq,
           Id_err, Iq_err,
           Vd, Vq, CCR1);
}


/* ================================================================
 * SCENARIO 1 — Normal ramp up
 * ----------------------------
 * Start from zero
 * Iq_ref = 5A
 * Watch Id → 0, Iq → 5
 * ================================================================ */
void scenario_normal_rampup()
{
    printf("\n========================================\n");
    printf("  Scenario 1: Normal Ramp Up\n");
    printf("  Iq_ref = %.1fA, motor at 50Hz\n", IQ_REF);
    printf("========================================\n");

    /* Initialise FOC */
    foc_init(1.0f, 10.0f,       /* Id PI gains */
             1.0f, 10.0f,       /* Iq PI gains */
             MOTOR_OMEGA);      /* motor speed */

    foc_set_Iq_ref(IQ_REF);
    mock_gpio_set_enable(true);

    /* Motor model state — starts at zero */
    float Id_motor = 0.0f;
    float Iq_motor = 0.0f;

    print_header();

    for (int step = 0; step < SIM_STEPS; step++)
    {
        /* Step 1 — Run FOC cycle */
        foc_step();

        /* Step 2 — Read what PI decided */
        float Id, Iq, Vd, Vq, theta;
        foc_get_state(Id, Iq, Vd, Vq, theta);

        /* Step 3 — Run motor model */
        motor_model(Vd, Vq, Id_motor, Iq_motor);

        /* Step 4 — Feed back into FOC */
        foc_set_feedback(Id_motor, Iq_motor);

        /* Step 5 — Print every N steps */
        if (step % PRINT_EVERY == 0)
        {
            print_state(step, Id_motor, Iq_motor, Vd, Vq, theta);
        }
    }

    /* Final state */
    printf("\nFinal state after %d steps:\n", SIM_STEPS);
    float Id, Iq, Vd, Vq, theta;
    foc_get_state(Id, Iq, Vd, Vq, theta);
    printf("Id = %.4f A (target = 0.0)\n",  Id_motor);
    printf("Iq = %.4f A (target = %.1f)\n", Iq_motor, IQ_REF);
    printf("Vd = %.4f V\n", Vd);
    printf("Vq = %.4f V\n", Vq);
}


/* ================================================================
 * SCENARIO 2 — STO fault mid run
 * --------------------------------
 * Run normally for 400 steps
 * STO triggers at step 400
 * FOC must stop immediately
 * CCR must freeze
 * ================================================================ */
void scenario_STO_fault()
{
    printf("\n========================================\n");
    printf("  Scenario 2: STO Fault Mid Run\n");
    printf("  STO triggers at step 400\n");
    printf("========================================\n");

    foc_init(1.0f, 10.0f, 1.0f, 10.0f, MOTOR_OMEGA);
    foc_set_Iq_ref(IQ_REF);
    mock_gpio_set_enable(true);

    float Id_motor = 0.0f;
    float Iq_motor = 0.0f;

    printf("\n%-6s %-8s %-8s %-8s %-12s\n",
           "Step", "Iq", "Vq", "CCR1", "Status");

    for (int step = 0; step < 600; step++)
    {
        /* Trigger STO at step 400 */
        if (step == 400)
        {
            mock_gpio_set_STO(true);
            printf("\n--- STO TRIGGERED at step 400 ---\n\n");
        }

        foc_step();

        float Id, Iq, Vd, Vq, theta;
        foc_get_state(Id, Iq, Vd, Vq, theta);

        motor_model(Vd, Vq, Id_motor, Iq_motor);
        foc_set_feedback(Id_motor, Iq_motor);

        /* Print at key points */
        if (step % 100 == 0 || step == 399 || step == 400 || step == 401)
        {
            uint32_t CCR1, CCR2, CCR3;
            mock_pwm_read(CCR1, CCR2, CCR3);

            const char *status = mock_gpio_read_STO() ? "STO ACTIVE" : "running";

            printf("%-6d %-8.3f %-8.3f %-8u %-12s\n",
                   step, Iq_motor, Vq, CCR1, status);
        }
    }
}


/* ================================================================
 * SCENARIO 3 — Speed reference change
 * -------------------------------------
 * Start at Iq_ref = 3A
 * Change to Iq_ref = 7A at step 500
 * Watch Iq track new reference
 * ================================================================ */
void scenario_speed_change()
{
    printf("\n========================================\n");
    printf("  Scenario 3: Speed Reference Change\n");
    printf("  Iq_ref changes 3A → 7A at step 500\n");
    printf("========================================\n");

    foc_init(1.0f, 10.0f, 1.0f, 10.0f, MOTOR_OMEGA);
    foc_set_Iq_ref(3.0f);
    mock_gpio_set_enable(true);

    float Id_motor = 0.0f;
    float Iq_motor = 0.0f;

    print_header();

    for (int step = 0; step < 1000; step++)
    {
        /* Change speed reference at step 500 */
        if (step == 500)
        {
            foc_set_Iq_ref(7.0f);
            printf("\n--- Iq_ref changed to 7.0A at step 500 ---\n\n");
        }

        foc_step();

        float Id, Iq, Vd, Vq, theta;
        foc_get_state(Id, Iq, Vd, Vq, theta);

        motor_model(Vd, Vq, Id_motor, Iq_motor);
        foc_set_feedback(Id_motor, Iq_motor);

        if (step % 100 == 0 || step == 499 || step == 500 || step == 501)
        {
            print_state(step, Id_motor, Iq_motor, Vd, Vq, theta);
        }
    }
}


/* ================================================================
 * MAIN
 * ================================================================ */
int main()
{
    printf("========================================\n");
    printf("  VFD FOC Full System Simulation\n");
    printf("  Motor: Rs=%.1f Ls=%.3f omega=%.1f\n",
           MOTOR_RS, MOTOR_LS, MOTOR_OMEGA);
    printf("========================================\n");

    scenario_normal_rampup();
    scenario_STO_fault();
    scenario_speed_change();

    printf("\n========================================\n");
    printf("  Simulation complete\n");
    printf("========================================\n");

    return 0;
}