#include "foc_controller.h"


/* ================================================================
 * STATIC STATE
 * Lives for lifetime of program — no dynamic memory
 * ================================================================ */

static PIController id_pi;          /* Flux PI controller     */
static PIController iq_pi;          /* Torque PI controller   */
static float        s_Iq_ref = 0.0f;/* Torque current ref     */

/* Last computed values — readable via foc_get_state() */
static float s_Id    = 0.0f;
static float s_Iq    = 0.0f;
static float s_Vd    = 0.0f;
static float s_Vq    = 0.0f;
static float s_theta = 0.0f;


/* ================================================================
 * foc_init
 * ================================================================ */

void foc_init(float Kp_id, float Ki_id,
              float Kp_iq, float Ki_iq,
              float omega_start)
{
    /* Initialise Id PI — flux controller
     * Output limits are voltage — fraction of Vbus
     * -300V to +300V — half DC bus each direction */
    pi_init(id_pi,
            Kp_id, Ki_id,
            FOC_DT,
            -300.0f, 300.0f);

    /* Initialise Iq PI — torque controller
     * Same voltage limits */
    pi_init(iq_pi,
            Kp_iq, Ki_iq,
            FOC_DT,
            -300.0f, 300.0f);

    /* Initialise mock HAL */
    mock_adc_init(FOC_PEAK_CURRENT);
    mock_encoder_init(omega_start, 0.0f);
    mock_timer_init(FOC_ARR, FOC_VBUS);

    /* GPIO starts disabled and clear — safe state */
    mock_gpio_set_enable(false);
    mock_gpio_set_STO(false);
    mock_gpio_set_fault(false);

    /* Zero internal state */
    s_Iq_ref = 0.0f;
    s_Id     = 0.0f;
    s_Iq     = 0.0f;
    s_Vd     = 0.0f;
    s_Vq     = 0.0f;
    s_theta  = 0.0f;
}


/* ================================================================
 * foc_set_Iq_ref
 * ================================================================ */

void foc_set_Iq_ref(float Iq_ref)
{
    s_Iq_ref = Iq_ref;
}


/* ================================================================
 * foc_step
 * ================================================================ */

void foc_step(void)
{
    /* --------------------------------------------------------
     * Step 0 — Safety check
     * Check before doing anything else
     * If not safe — return immediately, do not touch motor
     * -------------------------------------------------------- */
    if (!mock_gpio_read_enable())   return;   /* not enabled by PLC  */
    if (mock_gpio_read_STO())       return;   /* STO active — coast  */

    /* --------------------------------------------------------
     * Step 1 — Read encoder
     * Get current rotor angle
     * Compute cos and sin ONCE — both Park and Inverse Park
     * need the same values — no repeated trig calculation
     * -------------------------------------------------------- */
    float theta     = mock_encoder_read_theta();
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);

    /* --------------------------------------------------------
     * Step 2 — Read phase currents from ADC
     * Ia and Ib from mock — sine waves at current theta
     * Ic = -(Ia + Ib) by Kirchhoff — not needed explicitly
     * -------------------------------------------------------- */
    float Ia = mock_adc_read_Ia(theta);
    float Ib = mock_adc_read_Ib(theta);

    /* --------------------------------------------------------
     * Step 3 — Clarke transform
     * 3 phase stationary → 2 phase stationary
     * Ia, Ib → Ialpha, Ibeta
     * -------------------------------------------------------- */
    float Ialpha, Ibeta;
    clarke(Ia, Ib, Ialpha, Ibeta);

    /* --------------------------------------------------------
     * Step 4 — Park transform
     * 2 phase stationary → 2 phase rotating
     * Ialpha, Ibeta → Id, Iq
     * Id = flux current    → want near 0
     * Iq = torque current  → want near Iq_ref
     * -------------------------------------------------------- */
    float Id, Iq;
    park(Ialpha, Ibeta, cos_theta, sin_theta, Id, Iq);

    /* --------------------------------------------------------
     * Step 5 — PI controllers
     * Id error = 0 - Id         → id_pi → Vd
     * Iq error = Iq_ref - Iq    → iq_pi → Vq
     *
     * Vd and Vq are BORN HERE
     * They are the voltage demands in rotating frame
     * PI decides what voltage to apply to fix the error
     * -------------------------------------------------------- */
    float Vd = pi_update(id_pi, FOC_ID_REF - Id);
    float Vq = pi_update(iq_pi, s_Iq_ref   - Iq);

    /* --------------------------------------------------------
     * Step 6 — Inverse Park
     * 2 phase rotating → 2 phase stationary
     * Vd, Vq → Valpha, Vbeta
     * Same cos_theta sin_theta as Park — no extra trig cost
     * -------------------------------------------------------- */
    float Valpha, Vbeta;
    inverse_park(Vd, Vq, cos_theta, sin_theta, Valpha, Vbeta);

    /* --------------------------------------------------------
     * Step 7 — SVPWM
     * Voltage vector → PWM compare values
     * Valpha, Vbeta → CCR1, CCR2, CCR3
     * -------------------------------------------------------- */
    uint32_t CCR1, CCR2, CCR3;
    svpwm(Valpha, Vbeta, FOC_VBUS, FOC_ARR, CCR1, CCR2, CCR3);

    /* --------------------------------------------------------
     * Step 8 — Write CCR values to mock PWM
     * In real firmware: TIM1->CCR1 = CCR1 etc
     * Here: stored in mock HAL variables
     * -------------------------------------------------------- */
    mock_pwm_write(CCR1, CCR2, CCR3);

    /* --------------------------------------------------------
     * Step 9 — Advance encoder
     * Motor has rotated by omega*dt since last cycle
     * Update theta for next cycle
     * -------------------------------------------------------- */
    mock_encoder_step(FOC_DT);

    /* --------------------------------------------------------
     * Store state for foc_get_state()
     * -------------------------------------------------------- */
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

    s_Id    = 0.0f;
    s_Iq    = 0.0f;
    s_Vd    = 0.0f;
    s_Vq    = 0.0f;
}
