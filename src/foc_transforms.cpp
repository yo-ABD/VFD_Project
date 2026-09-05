#include "foc_transforms.h"


/**
 * clarke
 * ------
 * Power-invariant Clarke transformation.
 *
 * Derivation:
 *     3 phase axes are 120 degrees apart.
 *     Project onto 2 perpendicular axes (alpha, beta).
 *     Alpha axis aligned with phase A — so Ialpha = Ia directly.
 *     Beta axis is 90 degrees from alpha.
 *     Ibeta derived from projection of Ib and Ic onto beta axis.
 *     Ic eliminated using Kirchhoff: Ic = -(Ia + Ib)
 *     Result: Ibeta = (Ia + 2*Ib) / sqrt(3)
 *
 * No division at runtime — ONE_OVER_SQRT3 is compile time constant.
 */
void clarke(float Ia, float Ib,
            float &Ialpha, float &Ibeta)
{
    Ialpha = Ia;
    Ibeta  = (Ia + 2.0f * Ib) * ONE_OVER_SQRT3;
}


/**
 * park
 * ----
 * Rotation matrix by angle theta.
 * Rotates stationary alpha/beta frame into rotating d/q frame.
 *
 * Matrix form:
 *     [ Id ]   [  cos  sin ] [ Ialpha ]
 *     [ Iq ] = [ -sin  cos ] [ Ibeta  ]
 *
 * cos_theta and sin_theta precomputed by caller — no trig cost here.
 */
void park(float Ialpha, float Ibeta,
          float cos_theta, float sin_theta,
          float &Id, float &Iq)
{
    Id =  Ialpha * cos_theta + Ibeta * sin_theta;
    Iq = -Ialpha * sin_theta + Ibeta * cos_theta;
}


/**
 * inverse_park
 * ------------
 * Inverse rotation matrix — rotate back by theta.
 * Takes rotating d/q voltages back to stationary alpha/beta frame.
 *
 * Matrix form:
 *     [ Valpha ]   [ cos  -sin ] [ Vd ]
 *     [ Vbeta  ] = [ sin   cos ] [ Vq ]
 *
 * Same cos_theta and sin_theta as park() — no extra trig cost.
 */
void inverse_park(float Vd, float Vq,
                  float cos_theta, float sin_theta,
                  float &Valpha, float &Vbeta)
{
    Valpha = Vd * cos_theta - Vq * sin_theta;
    Vbeta  = Vd * sin_theta + Vq * cos_theta;
}
