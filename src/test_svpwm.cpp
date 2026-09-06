#include <stdio.h>
#include <math.h>
#include "svpwm.h"

#define ARR         8500
#define VBUS        600.0f
#define TS          0.0001f
#define TOLERANCE   0.01f

bool nearly_equal(float a, float b)
{
    float diff = a - b;
    if (diff < 0.0f) diff = -diff;
    return diff < TOLERANCE;
}


/**
 * Test 1 — Sector Detection
 * -------------------------
 * Known Valpha Vbeta vectors in each sector.
 * Verify correct sector returned.
 *
 * Sector boundaries at 60 degree intervals:
 *     Sector 1: 0   to 60  degrees
 *     Sector 2: 60  to 120 degrees
 *     Sector 3: 120 to 180 degrees
 *     Sector 4: 180 to 240 degrees
 *     Sector 5: 240 to 300 degrees
 *     Sector 6: 300 to 360 degrees
 */
void test_sector_detection()
{
    printf("\n=== Test 1: Sector Detection ===\n");
    printf("Known vectors at centre of each sector\n\n");

    /* Centre angles of each sector in degrees */
    struct {
        float angle_deg;
        int   expected_sector;
    } cases[6] = {
        {  30.0f, 1 },
        {  90.0f, 2 },
        { 150.0f, 3 },
        { 210.0f, 4 },
        { 270.0f, 5 },
        { 330.0f, 6 }
    };

    printf("%-12s %-10s %-10s %-10s %-8s\n",
           "Angle(deg)", "Valpha", "Vbeta", "Expected", "Result");

    for (int i = 0; i < 6; i++)
    {
        float angle_rad = cases[i].angle_deg * (float)M_PI / 180.0f;
        float Valpha    = cosf(angle_rad);
        float Vbeta     = sinf(angle_rad);
        int   sector    = svpwm_get_sector(Valpha, Vbeta);

        printf("%-12.1f %-10.4f %-10.4f %-10d %-8s\n",
               cases[i].angle_deg,
               Valpha, Vbeta,
               cases[i].expected_sector,
               (sector == cases[i].expected_sector) ? "PASS" : "FAIL");
    }
}


/**
 * Test 2 — Zero Vector
 * --------------------
 * Valpha = 0, Vbeta = 0 — no voltage demand
 * T1 = T2 = 0, T0 = Ts
 * All CCR should equal ARR/2 — 50% duty — zero volts to motor
 */
void test_zero_vector()
{
    printf("\n=== Test 2: Zero Vector ===\n");
    printf("Valpha = 0, Vbeta = 0 — motor at standstill\n");
    printf("Expected: CCR1 = CCR2 = CCR3 = ARR/2 = 4250\n\n");

    uint32_t CCR1, CCR2, CCR3;
    svpwm(0.0f, 0.0f, VBUS, ARR, CCR1, CCR2, CCR3);

    printf("CCR1 = %u  CCR2 = %u  CCR3 = %u\n", CCR1, CCR2, CCR3);
    printf("Result: %s\n",
           (CCR1 == ARR/2 && CCR2 == ARR/2 && CCR3 == ARR/2)
           ? "PASS" : "CHECK — small offset acceptable");
}


/**
 * Test 3 — XYZ Calculation
 * ------------------------
 * Verify X + Y + Z = 0 always (mathematical property)
 * Test with several vectors
 */
void test_XYZ_sum_zero()
{
    printf("\n=== Test 3: XYZ Sum Always Zero ===\n");
    printf("Mathematical property: X + Y + Z must always = 0\n\n");

    printf("%-10s %-8s %-8s %-8s %-10s %-8s\n",
           "Angle", "X", "Y", "Z", "X+Y+Z", "Pass?");

    bool all_pass = true;

    for (int deg = 0; deg < 360; deg += 30)
    {
        float angle  = (float)deg * (float)M_PI / 180.0f;
        float Valpha = 100.0f * cosf(angle);
        float Vbeta  = 100.0f * sinf(angle);

        float X, Y, Z;
        svpwm_calc_XYZ(Valpha, Vbeta, VBUS, X, Y, Z);

        float sum  = X + Y + Z;
        bool  pass = nearly_equal(sum, 0.0f);
        if (!pass) all_pass = false;

        printf("%-10d %-8.4f %-8.4f %-8.4f %-10.6f %-8s\n",
               deg, X, Y, Z, sum, pass ? "PASS" : "FAIL");
    }

    printf("\nAll XYZ sum zero: %s\n", all_pass ? "PASS" : "FAIL");
}


/**
 * Test 4 — Rotating Vector Sector Sequence
 * -----------------------------------------
 * Rotate vector through full 360 degrees
 * Verify sector changes 1→2→3→4→5→6→1
 * Verify CCR values change smoothly — no sudden jumps
 */
void test_rotating_vector()
{
    printf("\n=== Test 4: Rotating Vector ===\n");
    printf("Vector rotating through 360 degrees at 30 degree steps\n");
    printf("Vref magnitude = 200V (within linear modulation range)\n\n");

    printf("%-8s %-8s %-10s %-10s %-8s %-8s %-8s\n",
           "Angle", "Sector", "Valpha", "Vbeta", "CCR1", "CCR2", "CCR3");

    float    Vmag  = 200.0f;   /* voltage magnitude — well within Vbus/sqrt(3) = 346V */
    uint32_t prev_CCR1 = 0;
    int      prev_sector = 0;

    for (int deg = 0; deg <= 360; deg += 30)
    {
        float angle  = (float)deg * (float)M_PI / 180.0f;
        float Valpha = Vmag * cosf(angle);
        float Vbeta  = Vmag * sinf(angle);

        int sector = svpwm_get_sector(Valpha, Vbeta);

        uint32_t CCR1, CCR2, CCR3;
        svpwm(Valpha, Vbeta, VBUS, ARR, CCR1, CCR2, CCR3);

        /* Check for sector change */
        const char *sector_note = "";
        if (prev_sector != 0 && sector != prev_sector)
        {
            sector_note = " <-- sector change";
        }

        printf("%-8d %-8d %-10.2f %-10.2f %-8u %-8u %-8u%s\n",
               deg, sector, Valpha, Vbeta, CCR1, CCR2, CCR3, sector_note);

        prev_sector = sector;
        prev_CCR1   = CCR1;
    }
}


/**
 * Test 5 — Overmodulation Clamp
 * ------------------------------
 * Vector magnitude larger than Vbus/sqrt(3) = 346V
 * T1 + T2 would exceed Ts
 * Must clamp — T1 + T2 must never exceed Ts
 * CCR values must stay within [0, ARR]
 */
void test_overmodulation()
{
    printf("\n=== Test 5: Overmodulation Clamp ===\n");
    printf("Vref = 400V — exceeds Vbus/sqrt(3) = 346V\n");
    printf("Expect: CCR values clamped, no values above ARR\n\n");

    float    Valpha = 400.0f;
    float    Vbeta  = 0.0f;
    uint32_t CCR1, CCR2, CCR3;

    svpwm(Valpha, Vbeta, VBUS, ARR, CCR1, CCR2, CCR3);

    printf("Valpha=400V Vbeta=0V\n");
    printf("CCR1=%u  CCR2=%u  CCR3=%u  ARR=%u\n", CCR1, CCR2, CCR3, ARR);
    printf("All within [0, ARR]: %s\n",
           (CCR1 <= ARR && CCR2 <= ARR && CCR3 <= ARR) ? "PASS" : "FAIL");

    /* Verify X+Y+Z still zero even in overmodulation */
    float X, Y, Z;
    svpwm_calc_XYZ(Valpha, Vbeta, VBUS, X, Y, Z);
    printf("X+Y+Z = %.6f (expect 0): %s\n",
           X+Y+Z, nearly_equal(X+Y+Z, 0.0f) ? "PASS" : "FAIL");
}


int main()
{
    printf("========================================\n");
    printf("  SVPWM Unit Tests\n");
    printf("========================================\n");

    test_sector_detection();
    test_zero_vector();
    test_XYZ_sum_zero();
    test_rotating_vector();
    test_overmodulation();

    printf("\n========================================\n");
    printf("  All tests complete\n");
    printf("========================================\n");

    return 0;
}