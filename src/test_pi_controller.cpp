#include <stdio.h>
#include "pi_controller.h"

/**
 * Test 1 — Step input
 * -------------------
 * Reference = 10.0, feedback starts at 0.
 * Error starts at 10, should reduce toward 0 over time.
 * Output should rise and settle.
 */
void test_step_input()
{
    printf("\n=== Test 1: Step Input ===\n");
    printf("Reference = 10.0, feedback = 0.0 fixed\n");
    printf("Expect: output rises, error stays at 10 (no feedback loop here)\n\n");

    PIController pi;
    pi_init(pi, 1.0f, 10.0f, 0.0001f, -100.0f, 100.0f);

    float reference = 10.0f;
    float feedback  = 0.0f;

    printf("%-6s %-10s %-12s %-12s %-12s\n",
           "Step", "Error", "Integrator", "Output", "Saturated");

    for (int i = 0; i < 20; i++)
    {
        float error  = reference - feedback;
        float output = pi_update(pi, error);

        printf("%-6d %-10.4f %-12.4f %-12.4f %-12s\n",
               i,
               error,
               pi.integrator,
               output,
               (output >= 100.0f || output <= -100.0f) ? "YES" : "no");
    }
}


/**
 * Test 2 — Anti-windup test
 * -------------------------
 * Large error that pushes output to saturation.
 * Integrator should STOP growing when output is clamped.
 */
void test_anti_windup()
{
    printf("\n=== Test 2: Anti-Windup ===\n");
    printf("Large error = 100.0, output_max = 50.0\n");
    printf("Expect: output hits 50 and STAYS there, integrator stops growing\n\n");

    PIController pi;
    pi_init(pi, 1.0f, 100.0f, 0.0001f, -50.0f, 50.0f);

    float error = 100.0f;

    printf("%-6s %-12s %-12s\n", "Step", "Integrator", "Output");

    for (int i = 0; i < 10; i++)
    {
        float output = pi_update(pi, error);

        printf("%-6d %-12.4f %-12.4f\n",
               i,
               pi.integrator,
               output);
    }
}


/**
 * Test 3 — Reset test
 * -------------------
 * Run for 10 steps, then reset.
 * Integrator must be zero after reset.
 */
void test_reset()
{
    printf("\n=== Test 3: Reset ===\n");
    printf("Run 10 steps, then call pi_reset\n");
    printf("Expect: integrator and prev_error = 0 after reset\n\n");

    PIController pi;
    pi_init(pi, 1.0f, 10.0f, 0.0001f, -100.0f, 100.0f);

    for (int i = 0; i < 10; i++)
    {
        pi_update(pi, 5.0f);
    }

    printf("Before reset — integrator: %.4f  prev_error: %.4f\n",
           pi.integrator, pi.prev_error);

    pi_reset(pi);

    printf("After  reset — integrator: %.4f  prev_error: %.4f\n",
           pi.integrator, pi.prev_error);

    if (pi.integrator == 0.0f && pi.prev_error == 0.0f)
    {
        printf("PASS — reset cleared state correctly\n");
    }
    else
    {
        printf("FAIL — reset did not clear state\n");
    }
}


/**
 * Test 4 — Negative error
 * -----------------------
 * Reference below feedback — output should go negative.
 */
void test_negative_error()
{
    printf("\n=== Test 4: Negative Error ===\n");
    printf("Error = -10.0 fixed\n");
    printf("Expect: output goes negative, hits output_min\n\n");

    PIController pi;
    pi_init(pi, 1.0f, 10.0f, 0.0001f, -100.0f, 100.0f);

    printf("%-6s %-10s %-12s %-12s\n", "Step", "Error", "Integrator", "Output");

    for (int i = 0; i < 15; i++)
    {
        float output = pi_update(pi, -10.0f);

        printf("%-6d %-10.4f %-12.4f %-12.4f\n",
               i, -10.0f, pi.integrator, output);
    }
}


int main()
{
    printf("========================================\n");
    printf("  PI Controller Unit Tests\n");
    printf("========================================\n");

    test_step_input();
    test_anti_windup();
    test_reset();
    test_negative_error();

    printf("\n========================================\n");
    printf("  All tests complete\n");
    printf("========================================\n");

    return 0;
}

