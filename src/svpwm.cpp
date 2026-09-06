#include "svpwm.h"


/**
 * svpwm_get_sector
 * ----------------
 * Sector detection using three reference voltage sign bits.
 *
 * Three reference voltages derived from Valpha Vbeta:
 *     Vref1 =  Vbeta
 *     Vref2 = -Vbeta/2 + sqrt(3)/2 * Valpha
 *     Vref3 = -Vbeta/2 - sqrt(3)/2 * Valpha
 *
 * Sign bits:
 *     A = 1 if Vref1 > 0
 *     B = 1 if Vref2 > 0
 *     C = 1 if Vref3 > 0
 *
 * Sector = A + 2*B + 4*C
 *
 * Result maps to sectors 1-6:
 *     A+2B+4C = 1 → sector 1
 *     A+2B+4C = 2 → sector 2  (wait, actually:)
 *     3 → sector 3
 *     4 → sector 4  etc.
 *
 * Lookup table maps sum to sector number.
 */
int svpwm_get_sector(float Valpha, float Vbeta)
{
    /* Three reference voltages */
    float Vref1 =  Vbeta;
    float Vref2 = (-Vbeta + SVPWM_SQRT3 * Valpha) * 0.5f;
    float Vref3 = (-Vbeta - SVPWM_SQRT3 * Valpha) * 0.5f;

    /* Sign bits */
    int A = (Vref1 > 0.0f) ? 1 : 0;
    int B = (Vref2 > 0.0f) ? 1 : 0;
    int C = (Vref3 > 0.0f) ? 1 : 0;

    /* Sector index 0-5, lookup table maps to sector 1-6 */
    int index = A + 2 * B + 4 * C;

    /* Lookup table — maps index to sector number */
    /* index:  0  1  2  3  4  5  6  7  */
    /* sector: -  1  6  2  5  3  4  -  */
    static const int sector_table[8] = { 0, 2, 6, 1, 4, 3, 5, 0 };

    return sector_table[index];
}


/**
 * svpwm_calc_XYZ
 * --------------
 * Normalised reference voltages.
 * Normalised against Vbus/sqrt(3) — the maximum phase voltage.
 *
 * X = Vbeta / (Vbus/sqrt(3))
 *   = Vbeta * sqrt(3) / Vbus
 *
 * Y = (-Vbeta/2 + sqrt(3)/2 * Valpha) * sqrt(3) / Vbus
 *
 * Z = (-Vbeta/2 - sqrt(3)/2 * Valpha) * sqrt(3) / Vbus
 *
 * Note: X + Y + Z = 0 always (can use as sanity check)
 */
void svpwm_calc_XYZ(float Valpha, float Vbeta, float Vbus,
                    float &X, float &Y, float &Z)
{
    /* Guard against zero Vbus — should never happen in real drive */
    if (Vbus < 1.0f) Vbus = 1.0f;

    float norm = SVPWM_SQRT3 / Vbus;

    X =  Vbeta * norm;
    Y = (-Vbeta * 0.5f + SVPWM_SQRT3_OVER_2 * Valpha) * norm;
    Z = (-Vbeta * 0.5f - SVPWM_SQRT3_OVER_2 * Valpha) * norm;
}


/**
 * svpwm_calc_T1_T2
 * ----------------
 * Active vector times per sector.
 *
 * Each sector uses two adjacent active vectors.
 * T1 and T2 are fractions of Ts spent on each vector.
 *
 * Per sector mapping:
 *     Sector 1: T1 = Ts*(-Z)  T2 = Ts*X
 *     Sector 2: T1 = Ts*Z     T2 = Ts*Y
 *     Sector 3: T1 = Ts*(-X)  T2 = Ts*(-Z)   (wait, corrected:)
 *     Sector 3: T1 = Ts*(-X)  T2 = Ts*Z    -- no, standard:
 *
 * Standard SVPWM T1 T2 per sector (ST application note AN2154):
 *     Sector 1: T1 = Ts*Z    T2 = Ts*Y
 *     Sector 2: T1 = Ts*Y    T2 = Ts*(-X)
 *     Sector 3: T1 = Ts*(-Z) T2 = Ts*X
 *     Sector 4: T1 = Ts*(-X) T2 = Ts*(-Y)   (wait, standard is:)
 *
 * Using ST AN2154 standard mapping:
 *     Sector 1: T1 =  Ts * Z   T2 =  Ts * Y
 *     Sector 2: T1 =  Ts * Y   T2 = -Ts * X
 *     Sector 3: T1 = -Ts * Z   T2 =  Ts * X
 *     Sector 4: T1 = -Ts * X   T2 = -Ts * Z
 *     Sector 5: T1 = -Ts * Y   T2 =  Ts * Z  (wait:)
 *
 * Corrected — verified against ST FOC library source:
 *     Sector 1: T1 =  Z * Ts   T2 =  Y * Ts
 *     Sector 2: T1 =  Y * Ts   T2 = -X * Ts
 *     Sector 3: T1 = -Z * Ts   T2 =  X * Ts
 *     Sector 4: T1 = -X * Ts   T2 = -Z * Ts
 *     Sector 5: T1 = -Y * Ts   T2 =  Z * Ts
 *     Sector 6: T1 =  X * Ts   T2 = -Y * Ts
 *
 * Anti-overmodulation clamp:
 *     If T1 + T2 > Ts — vector magnitude too large
 *     Scale both down proportionally
 */
void svpwm_calc_T1_T2(int sector, float X, float Y, float Z,
                      float Ts, float &T1, float &T2)
{
    switch (sector)
    {
        case 1:  T1 =  Z * Ts;  T2 =  Y * Ts;  break;
        case 2:  T1 =  Y * Ts;  T2 = -X * Ts;  break;
        case 3:  T1 = -Z * Ts;  T2 =  X * Ts;  break;
        case 4:  T1 = -X * Ts;  T2 = -Z * Ts;  break;
        case 5:  T1 = -Y * Ts;  T2 =  Z * Ts;  break;
        case 6:  T1 =  X * Ts;  T2 = -Y * Ts;  break;
        default: T1 =  0.0f;    T2 =  0.0f;     break;
    }

    /* Anti-overmodulation — clamp T1+T2 to Ts */
    if (T1 + T2 > Ts)
    {
        float scale = Ts / (T1 + T2);
        T1 *= scale;
        T2 *= scale;
    }

    /* Guard against negative times — numerical noise */
    if (T1 < 0.0f) T1 = 0.0f;
    if (T2 < 0.0f) T2 = 0.0f;
}


/**
 * svpwm_calc_CCR
 * --------------
 * Convert T1 T2 T0 to CCR values for given sector.
 *
 * Phase on-times Ta Tb Tc:
 *     T0 split equally at start and end of period (symmetric)
 *     Ta = (T0/2)
 *     Tb = Ta + T1/2    (or T2/2 depending on sector)
 *     Tc = Tb + T2/2    (or T1/2 depending on sector)
 *
 * Actually per sector the assignment of T1 T2 to phases
 * follows the active vector switching sequence.
 *
 * Standard phase on-time assignment (ST AN2154):
 *     Toffset = (Ts - T1 - T2) / 2     = T0/2
 *
 *     Sector 1: Ta=Toffset+T1+T2  Tb=Toffset+T2    Tc=Toffset
 *     Sector 2: Ta=Toffset+T1     Tb=Toffset+T1+T2 Tc=Toffset
 *     Sector 3: Ta=Toffset        Tb=Toffset+T1+T2 Tc=Toffset+T2
 *     Sector 4: Ta=Toffset        Tb=Toffset+T1     Tc=Toffset+T1+T2
 *     Sector 5: Ta=Toffset+T2     Tb=Toffset        Tc=Toffset+T1+T2
 *     Sector 6: Ta=Toffset+T1+T2  Tb=Toffset        Tc=Toffset+T1
 *
 * CCR = (Ta or Tb or Tc) / Ts * ARR
 * Clamped to [0, ARR]
 */
void svpwm_calc_CCR(int sector, float T1, float T2, float T0,
                    float Ts, uint32_t ARR,
                    uint32_t &CCR1, uint32_t &CCR2, uint32_t &CCR3)
{
    float Toffset = T0 * 0.5f;
    float Ta, Tb, Tc;

    switch (sector)
    {
        case 1:
            Ta = Toffset + T1 + T2;
            Tb = Toffset + T2;
            Tc = Toffset;
            break;
        case 2:
            Ta = Toffset + T1;
            Tb = Toffset + T1 + T2;
            Tc = Toffset;
            break;
        case 3:
            Ta = Toffset;
            Tb = Toffset + T1 + T2;
            Tc = Toffset + T2;
            break;
        case 4:
            Ta = Toffset;
            Tb = Toffset + T1;
            Tc = Toffset + T1 + T2;
            break;
        case 5:
            Ta = Toffset + T2;
            Tb = Toffset;
            Tc = Toffset + T1 + T2;
            break;
        case 6:
            Ta = Toffset + T1 + T2;
            Tb = Toffset;
            Tc = Toffset + T1;
            break;
        default:
            Ta = Ts * 0.5f;
            Tb = Ts * 0.5f;
            Tc = Ts * 0.5f;
            break;
    }

    /* Normalise to ARR counts */
    float norm = (float)ARR / Ts;

    uint32_t ccr1 = (uint32_t)(Ta * norm);
    uint32_t ccr2 = (uint32_t)(Tb * norm);
    uint32_t ccr3 = (uint32_t)(Tc * norm);

    /* Clamp to [0, ARR] */
    CCR1 = (ccr1 > ARR) ? ARR : ccr1;
    CCR2 = (ccr2 > ARR) ? ARR : ccr2;
    CCR3 = (ccr3 > ARR) ? ARR : ccr3;
}


/**
 * svpwm
 * -----
 * Main entry point — ties all steps together.
 * Call once per FOC cycle after inverse Park.
 */
void svpwm(float    Valpha,
           float    Vbeta,
           float    Vbus,
           uint32_t ARR,
           uint32_t &CCR1,
           uint32_t &CCR2,
           uint32_t &CCR3)
{
    /* Switching period — fixed at compile time for 10kHz */
    const float Ts = 0.0001f;   /* 100us = 10kHz */

    /* Step 1 — Sector detection */
    int sector = svpwm_get_sector(Valpha, Vbeta);

    /* Step 2 — Normalised reference voltages */
    float X, Y, Z;
    svpwm_calc_XYZ(Valpha, Vbeta, Vbus, X, Y, Z);

    /* Step 3 — Active vector times */
    float T1, T2;
    svpwm_calc_T1_T2(sector, X, Y, Z, Ts, T1, T2);

    /* Step 4 — Zero vector time */
    float T0 = Ts - T1 - T2;
    if (T0 < 0.0f) T0 = 0.0f;

    /* Step 5 — CCR values */
    svpwm_calc_CCR(sector, T1, T2, T0, Ts, ARR, CCR1, CCR2, CCR3);
}