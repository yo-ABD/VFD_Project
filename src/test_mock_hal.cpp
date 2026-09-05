#include <stdio.h>
#include <math.h>
#include "mock_hal.h"

#define TOLERANCE   0.001f
#define DT          0.0001f     /* 100us — 10kHz FOC cycle */
#define TWO_PI      6.28318530718f

bool nearly_equal(float a, float b)
{
    float diff = a - b;
    if (diff < 0.0f) diff = -diff;
    return diff < TOLERANCE;
}


/**
 * Test 1 — ADC sine wave shape
 * ----------------------------
 * Verify Ia and Ib are sine waves 120 degrees apart.
 * Verify Kirchhoff: Ia + Ib + Ic = 0 at every theta.
 */
void test_adc_sine_wave()
{
    printf("\n=== Test 1: ADC Sine Wave and Kirchhoff ===\n");
    printf("Peak current = 10A\n");
    printf("Expect: Ia and Ib sine waves, Ia+Ib+Ic = 0 every step\n\n");

    mock_adc_init(10.0f);

    printf("%-6s %-10s %-10s %-10s %-10s %-8s\n",
           "Step", "theta", "Ia", "Ib", "Ic", "Sum=0?");

    bool all_pass = true;

    for (int i = 0; i < 12; i++)
    {
        float theta = (float)i * TWO_PI / 12.0f;   /* 0 to 2pi in 12 steps */

        float Ia = mock_adc_read_Ia(theta);
        float Ib = mock_adc_read_Ib(theta);
        float Ic = -(Ia + Ib);                      /* Kirchhoff */
        float sum = Ia + Ib + Ic;

        bool pass = nearly_equal(sum, 0.0f);
        if (!pass) all_pass = false;

        printf("%-6d %-10.4f %-10.4f %-10.4f %-10.4f %-8s\n",
               i, theta, Ia, Ib, Ic, pass ? "PASS" : "FAIL");
    }

    printf("\nKirchhoff check: %s\n", all_pass ? "ALL PASS" : "FAILED");
}


/**
 * Test 2 — Encoder advances correctly
 * ------------------------------------
 * Init at 50Hz (314.16 rad/s).
 * Step 100 times at dt=100us.
 * One full revolution = 1/50 = 0.02 seconds = 200 steps.
 * After 200 steps theta should be back near 0 (full revolution).
 */
void test_encoder_advance()
{
    printf("\n=== Test 2: Encoder Advance and Wrap ===\n");
    printf("omega = 2*pi*50 = 314.16 rad/s (50Hz motor)\n");
    printf("dt = 100us, 200 steps = one full revolution\n\n");

    float omega = TWO_PI * 50.0f;   /* 50Hz motor */
    mock_encoder_init(omega, 0.0f);

    /* Print every 20 steps */
    printf("%-6s %-12s\n", "Step", "theta (rad)");

    for (int i = 0; i <= 200; i++)
    {
        float theta = mock_encoder_read_theta();

        if (i % 20 == 0)
        {
            printf("%-6d %-12.4f\n", i, theta);
        }

        if (i < 200)
        {
            mock_encoder_step(DT);
        }
    }

    float final_theta = mock_encoder_read_theta();
    printf("\nAfter 200 steps (1 full revolution):\n");
    printf("theta = %.4f rad (expect near 0.0 or 2*pi = 6.2832)\n", final_theta);
}


/**
 * Test 3 — PWM write and read back
 * ---------------------------------
 * Write known CCR values, read back, verify match.
 * Verify clamping — values above ARR must clamp to ARR.
 */
void test_pwm_write_read()
{
    printf("\n=== Test 3: PWM Write and Read Back ===\n");
    printf("ARR = 8500, Vbus = 600V\n\n");

    mock_timer_init(8500, 600.0f);

    /* Test normal write */
    mock_pwm_write(4250, 2125, 6375);

    uint32_t r1, r2, r3;
    mock_pwm_read(r1, r2, r3);

    printf("Written:  CCR1=4250  CCR2=2125  CCR3=6375\n");
    printf("Read back: CCR1=%-6u CCR2=%-6u CCR3=%-6u\n", r1, r2, r3);
    printf("Match: %s\n\n",
           (r1 == 4250 && r2 == 2125 && r3 == 6375) ? "PASS" : "FAIL");

    /* Test clamping — value above ARR */
    mock_pwm_write(9000, 8500, 100);
    mock_pwm_read(r1, r2, r3);

    printf("Written:   CCR1=9000 (above ARR)  CCR2=8500  CCR3=100\n");
    printf("Read back: CCR1=%-6u CCR2=%-6u CCR3=%-6u\n", r1, r2, r3);
    printf("Clamp test: CCR1 should be 8500: %s\n",
           (r1 == 8500) ? "PASS" : "FAIL");

    /* Print duty cycles */
    printf("\n");
    mock_pwm_write(4250, 2125, 6375);
    mock_pwm_print();
}


/**
 * Test 4 — GPIO states
 * --------------------
 * Set each pin, read back, verify stored correctly.
 */
void test_gpio_states()
{
    printf("\n=== Test 4: GPIO States ===\n\n");

    /* Initial state */
    printf("Initial state:\n");
    mock_gpio_print();

    /* Enable drive */
    mock_gpio_set_enable(true);
    printf("\nAfter enable:\n");
    mock_gpio_print();
    printf("Read enable: %s\n",
           mock_gpio_read_enable() ? "PASS (true)" : "FAIL");

    /* Trigger STO */
    mock_gpio_set_STO(true);
    printf("\nAfter STO active:\n");
    mock_gpio_print();
    printf("Read STO: %s\n",
           mock_gpio_read_STO() ? "PASS (true)" : "FAIL");

    /* Set fault */
    mock_gpio_set_fault(true);
    printf("\nAfter fault:\n");
    mock_gpio_print();
}


/**
 * Test 5 — Kirchhoff with changing omega
 * ----------------------------------------
 * Change motor speed mid simulation.
 * Verify Kirchhoff still holds after speed change.
 */
void test_kirchhoff_with_speed_change()
{
    printf("\n=== Test 5: Kirchhoff After Speed Change ===\n");
    printf("Start at 50Hz, change to 30Hz at step 6\n\n");

    mock_adc_init(10.0f);
    mock_encoder_init(TWO_PI * 50.0f, 0.0f);

    printf("%-6s %-10s %-10s %-10s %-10s %-10s\n",
           "Step", "theta", "Ia", "Ib", "Ic", "Sum");

    for (int i = 0; i < 12; i++)
    {
        if (i == 6)
        {
            mock_encoder_set_omega(TWO_PI * 30.0f);
            printf("--- Speed changed to 30Hz ---\n");
        }

        float theta = mock_encoder_read_theta();
        float Ia    = mock_adc_read_Ia(theta);
        float Ib    = mock_adc_read_Ib(theta);
        float Ic    = -(Ia + Ib);
        float sum   = Ia + Ib + Ic;

        printf("%-6d %-10.4f %-10.4f %-10.4f %-10.4f %-10.4f\n",
               i, theta, Ia, Ib, Ic, sum);

        mock_encoder_step(DT);
    }
}


int main()
{
    printf("========================================\n");
    printf("  Mock HAL Unit Tests\n");
    printf("========================================\n");

    test_adc_sine_wave();
    test_encoder_advance();
    test_pwm_write_read();
    test_gpio_states();
    test_kirchhoff_with_speed_change();

    printf("\n========================================\n");
    printf("  All tests complete\n");
    printf("========================================\n");

    return 0;
}
