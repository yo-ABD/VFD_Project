#include <stdio.h>
#include <math.h>
#include "foc_transforms.h"

/* Tolerance for float comparison */
#define TOLERANCE 0.0001f

/* Helper — compare two floats within tolerance */
bool nearly_equal(float a, float b)
{
    float diff = a - b;
    if (diff < 0.0f) diff = -diff;
    return diff < TOLERANCE;
}

/* Helper — print pass or fail */
void check(const char *name, float expected, float actual)
{
    if (nearly_equal(expected, actual))
    {
        printf("  PASS  %-20s  expected: %8.4f  got: %8.4f\n",
               name, expected, actual);
    }
    else
    {
        printf("  FAIL  %-20s  expected: %8.4f  got: %8.4f  diff: %8.4f\n",
               name, expected, actual, actual - expected);
    }
}


/**
 * Test 1 — Clarke at theta = 0
 * ----------------------------
 * Balanced 3 phase at angle zero:
 *     Ia =  1.0
 *     Ib = -0.5
 *     Ic = -0.5   (Kirchhoff: -(1.0 + (-0.5)) = -0.5 correct)
 *
 * Expected:
 *     Ialpha = Ia = 1.0
 *     Ibeta  = (Ia + 2*Ib) / sqrt(3) = (1.0 + 2*(-0.5)) / sqrt(3)
 *            = (1.0 - 1.0) / sqrt(3) = 0.0
 */
void test_clarke_theta_zero()
{
    printf("\n=== Test 1: Clarke at theta = 0 ===\n");
    printf("Ia = 1.0, Ib = -0.5 (balanced, angle zero)\n");
    printf("Expected: Ialpha = 1.0, Ibeta = 0.0\n\n");

    float Ialpha, Ibeta;
    clarke(1.0f, -0.5f, Ialpha, Ibeta);

    check("Ialpha", 1.0f, Ialpha);
    check("Ibeta",  0.0f, Ibeta);
}


/**
 * Test 2 — Clarke at theta = 90 degrees (pi/2)
 * ---------------------------------------------
 * Balanced 3 phase at 90 degrees:
 *     Ia =  0.0
 *     Ib = -sqrt(3)/2 = -0.8660
 *     Ic =  sqrt(3)/2 =  0.8660
 *
 * Expected:
 *     Ialpha = 0.0
 *     Ibeta  = (0.0 + 2*(-0.8660)) / sqrt(3)
 *            = -1.7320 / 1.7320 = -1.0
 */
void test_clarke_theta_90()
{
    printf("\n=== Test 2: Clarke at theta = 90 degrees ===\n");
    printf("Ia = 0.0, Ib = -0.8660 (balanced, angle 90)\n");
    printf("Expected: Ialpha = 0.0, Ibeta = -1.0\n\n");

    float Ia     =  0.0f;
    float Ib     = -0.86602540378f;
    float Ialpha, Ibeta;

    clarke(Ia, Ib, Ialpha, Ibeta);

    check("Ialpha", 0.0f,  Ialpha);
    check("Ibeta", -1.0f,  Ibeta);
}


/**
 * Test 3 — Park at theta = 0
 * --------------------------
 * At theta = 0: cos = 1, sin = 0
 * Rotation matrix becomes identity-like:
 *     Id =  Ialpha * 1 + Ibeta * 0 = Ialpha
 *     Iq = -Ialpha * 0 + Ibeta * 1 = Ibeta
 *
 * Input:  Ialpha = 1.0, Ibeta = 0.5
 * Expected: Id = 1.0, Iq = 0.5
 */
void test_park_theta_zero()
{
    printf("\n=== Test 3: Park at theta = 0 ===\n");
    printf("Ialpha = 1.0, Ibeta = 0.5, theta = 0\n");
    printf("Expected: Id = 1.0, Iq = 0.5\n\n");

    float theta     = 0.0f;
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    float Id, Iq;

    park(1.0f, 0.5f, cos_theta, sin_theta, Id, Iq);

    check("Id", 1.0f, Id);
    check("Iq", 0.5f, Iq);
}


/**
 * Test 4 — Inverse Park round trip
 * ---------------------------------
 * Apply Inverse Park then Park with same theta.
 * Result must equal original Vd, Vq.
 * Proves transforms are exact inverses of each other.
 *
 * Input:  Vd = 10.0, Vq = 5.0, theta = 0.785 (45 degrees)
 * Steps:
 *     Inverse Park → Valpha, Vbeta
 *     Park(Valpha, Vbeta) → Id, Iq
 *     Id must equal Vd, Iq must equal Vq
 */
void test_inverse_park_round_trip()
{
    printf("\n=== Test 4: Inverse Park Round Trip ===\n");
    printf("Vd = 10.0, Vq = 5.0, theta = 45 degrees\n");
    printf("Inverse Park then Park must recover original Vd and Vq\n\n");

    float Vd        = 10.0f;
    float Vq        =  5.0f;
    float theta     =  0.78539816339f;   /* pi/4 = 45 degrees */
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);

    /* Step 1 — Inverse Park */
    float Valpha, Vbeta;
    inverse_park(Vd, Vq, cos_theta, sin_theta, Valpha, Vbeta);

    printf("  Intermediate: Valpha = %.4f  Vbeta = %.4f\n\n", Valpha, Vbeta);

    /* Step 2 — Park back */
    float Id_recovered, Iq_recovered;
    park(Valpha, Vbeta, cos_theta, sin_theta, Id_recovered, Iq_recovered);

    check("Vd recovered", Vd, Id_recovered);
    check("Vq recovered", Vq, Iq_recovered);
}


/**
 * Test 5 — Rotating sine wave input becomes DC after Park
 * -------------------------------------------------------
 * This is the BIG proof — the whole point of FOC transforms.
 *
 * Feed balanced 3 phase sine currents:
 *     Ia = cos(theta)
 *     Ib = cos(theta - 2pi/3)
 *
 * Advance theta each step — motor spinning
 * After Clarke and Park:
 *     Id should stay near 0.0    (no flux component in our fake signal)
 *     Iq should stay near 1.0    (constant torque component)
 *
 * If transforms are correct — DC output regardless of theta
 * If transforms are broken  — Id and Iq oscillate as sine waves
 */
void test_rotating_to_dc()
{
    printf("\n=== Test 5: Rotating Sine Wave Becomes DC ===\n");
    printf("Fake 3ph sine currents rotating with theta\n");
    printf("Expected: Id near 0.0, Iq near 1.0 every step\n\n");

    printf("%-6s %-10s %-10s %-10s %-10s %-10s\n",
           "Step", "theta", "Ia", "Ib", "Id", "Iq");

    float theta = 0.0f;

    for (int i = 0; i < 12; i++)
    {
        /* Fake balanced 3 phase currents */
        float Ia = cosf(theta);
        float Ib = cosf(theta - 2.0f * (float)M_PI / 3.0f);

        /* Clarke */
        float Ialpha, Ibeta;
        clarke(Ia, Ib, Ialpha, Ibeta);

        /* Park */
        float cos_theta = cosf(theta);
        float sin_theta = sinf(theta);
        float Id, Iq;
        park(Ialpha, Ibeta, cos_theta, sin_theta, Id, Iq);

        printf("%-6d %-10.4f %-10.4f %-10.4f %-10.4f %-10.4f\n",
               i, theta, Ia, Ib, Id, Iq);

        /* Advance theta — motor spinning */
        theta += (float)M_PI / 6.0f;   /* 30 degrees per step */
    }

    printf("\nIf Id stays near 0.0 and Iq stays near 1.0 — PASS\n");
    printf("If Id and Iq oscillate — transforms are broken\n");
}


int main()
{
    printf("========================================\n");
    printf("  FOC Transforms Unit Tests\n");
    printf("========================================\n");

    test_clarke_theta_zero();
    test_clarke_theta_90();
    test_park_theta_zero();
    test_inverse_park_round_trip();
    test_rotating_to_dc();

    printf("\n========================================\n");
    printf("  All tests complete\n");
    printf("========================================\n");

    return 0;
}
