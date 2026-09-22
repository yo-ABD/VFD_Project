#include "foc_controller.h"
#include "drive_state.h"
#include "fault_manager.h"


/* ================================================================
 * STATIC STATE
 * Lives for lifetime of program — no dynamic memory
 * ================================================================ */

static PIController id_pi;
static PIController iq_pi;
static float        s_Iq_ref         = 0.0f;
static float        s_Id             = 0.0f;
static float        s_Iq             = 0.0f;
static float        s_Vd             = 0.0f;
static float        s_Vq             = 0.0f;
static float        s_theta          = 0.0f;
static bool         use_ext_feedback = false;
static float        ext_Id           = 0.0f;
static float        ext_Iq           = 0.0f;


/* ================================================================
 * foc_init
 * ================================================================ */

void foc_init(float Kp_id, float Ki_id,
              float Kp_iq, float Ki_iq,
              float omega_start)
{
    pi_init(id_pi, Kp_id, Ki_id, FOC_DT, -300.0f, 300.0f);
    pi_init(iq_pi, Kp_iq, Ki_iq, FOC_DT, -300.0f, 300.0f);

    mock_adc_init(FOC_PEAK_CURRENT);
    mock_encoder_init(omega_start, 0.0f);
    mock_timer_init(FOC_ARR, FOC_VBUS);

    mock_gpio_set_enable(false);
    mock_gpio_set_STO(false);
    mock_gpio_set_fault(false);

    s_Iq_ref         = 0.0f;
    s_Id             = 0.0f;
    s_Iq             = 0.0f;
    s_Vd             = 0.0f;
    s_Vq             = 0.0f;
    s_theta          = 0.0f;
    use_ext_feedback = false;
    ext_Id           = 0.0f;
    ext_Iq           = 0.0f;
}


/* ================================================================
 * foc_set_Iq_ref
 * ================================================================ */

void foc_set_Iq_ref(float Iq_ref)
{
    s_Iq_ref = Iq_ref;
}


/* ================================================================
 * foc_set_feedback
 * ================================================================ */

void foc_set_feedback(float Id, float Iq)
{
    ext_Id           = Id;
    ext_Iq           = Iq;
    use_ext_feedback = true;
}


/* ================================================================
 * foc_step
 * ================================================================ */

void foc_step(void)
{
    /* Step 0 — Safety check */
    if (!mock_gpio_read_enable()) return;
    if (mock_gpio_read_STO())     return;
    if (drive_get_state() != STATE_RUNNING)    return;

    /* Step 1 — Read encoder */
    float theta     = mock_encoder_read_theta();
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);

    /* Steps 2 3 4 — Get Id and Iq */
    float Id, Iq;

    if (use_ext_feedback)
    {
        Id = ext_Id;
        Iq = ext_Iq;
    }
    else
    {
        float Ia = mock_adc_read_Ia(theta);
        float Ib = mock_adc_read_Ib(theta);

        /* Overcurrent check */
        const DriveParams* params = drive_get_params();
        float Ia_abs = Ia < 0.0f ? -Ia : Ia;

        if (Ia_abs > params->overcurrent_limit)
        {
            fault_log(FAULT_OVERCURRENT,
                    mock_timer_get_Vbus(),
                    Ia, s_Id, s_Iq, 0.0f);
            drive_transition(STATE_FAULT_LOCKOUT);
            return;
        }

        float Ialpha, Ibeta;
        clarke(Ia, Ib, Ialpha, Ibeta);
        park(Ialpha, Ibeta, cos_theta, sin_theta, Id, Iq);
    }

    /* Step 5 — PI controllers */
    float Vd = pi_update(id_pi, FOC_ID_REF - Id);
    float Vq = pi_update(iq_pi, s_Iq_ref   - Iq);

    /* Decoupling feedforward
     * Cancels cross coupling between d and q axes
     * Without this: Id and Iq fight each other
     * With this: each PI controls its axis independently */
    const float MOTOR_LS    = 0.005f;
    const float MOTOR_OMEGA = FOC_OMEGA_DEFAULT;
    Vd = Vd - MOTOR_OMEGA * MOTOR_LS * Iq;
    Vq = Vq + MOTOR_OMEGA * MOTOR_LS * Id;

    /* Step 6 — Inverse Park */
    float Valpha, Vbeta;
    inverse_park(Vd, Vq, cos_theta, sin_theta, Valpha, Vbeta);

    /* Step 7 — SVPWM */
    uint32_t CCR1, CCR2, CCR3;
    svpwm(Valpha, Vbeta, FOC_VBUS, FOC_ARR, CCR1, CCR2, CCR3);

    /* Step 8 — Write CCR */
    mock_pwm_write(CCR1, CCR2, CCR3);

    /* Step 9 — Advance encoder */
    mock_encoder_step(FOC_DT);

    /* Store state */
    s_Id    = Id;
    s_Iq    = Iq;
    s_Vd    = Vd;
    s_Vq    = Vq;
    s_theta = theta;
}


/* ================================================================
 * foc_get_state
 * ================================================================ */

void foc_get_state(float &Id, float &Iq,
                   float &Vd, float &Vq,
                   float &theta)
{
    Id    = s_Id;
    Iq    = s_Iq;
    Vd    = s_Vd;
    Vq    = s_Vq;
    theta = s_theta;
}


/* ================================================================
 * foc_reset
 * ================================================================ */

void foc_reset(void)
{
    pi_reset(id_pi);
    pi_reset(iq_pi);

    use_ext_feedback = false;
    ext_Id           = 0.0f;
    ext_Iq           = 0.0f;
    s_Id             = 0.0f;
    s_Iq             = 0.0f;
    s_Vd             = 0.0f;
    s_Vq             = 0.0f;
}