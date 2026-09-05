#include "mock_hal.h"


/* ================================================================
 * STATIC STATE — all hardware state lives here
 * Static = exists for lifetime of program
 * No dynamic memory — no malloc, no new
 * ================================================================ */

/* ADC state */
static float adc_peak_current  = 10.0f;    /* amps */

/* Encoder state */
static float enc_theta          = 0.0f;    /* radians */
static float enc_omega          = 0.0f;    /* rad/s   */

/* PWM Timer state */
static uint32_t tim_ARR         = 8500;    /* 10kHz at 170MHz centre aligned */
static float    tim_Vbus        = 600.0f;  /* DC bus volts */
static uint32_t tim_CCR1        = 0;
static uint32_t tim_CCR2        = 0;
static uint32_t tim_CCR3        = 0;

/* GPIO state */
static bool gpio_STO_active     = false;
static bool gpio_fault          = false;
static bool gpio_enable         = false;


/* ================================================================
 * ADC
 * ================================================================ */

void mock_adc_init(float peak_amps)
{
    adc_peak_current = peak_amps;
}

float mock_adc_read_Ia(float theta)
{
    /* Phase A — reference phase, no offset */
    return adc_peak_current * sinf(theta);
}

float mock_adc_read_Ib(float theta)
{
    /* Phase B — 120 degrees (2pi/3) behind phase A */
    return adc_peak_current * sinf(theta - MOCK_TWO_PI_3);
}


/* ================================================================
 * ENCODER
 * ================================================================ */

void mock_encoder_init(float omega_rad_per_sec, float theta_start)
{
    enc_omega = omega_rad_per_sec;
    enc_theta = theta_start;
}

float mock_encoder_step(float dt)
{
    /* Advance theta by one time step */
    enc_theta += enc_omega * dt;

    /* Wrap theta to [0, 2*pi] */
    if (enc_theta >= MOCK_TWO_PI)
    {
        enc_theta -= MOCK_TWO_PI;
    }
    if (enc_theta < 0.0f)
    {
        enc_theta += MOCK_TWO_PI;
    }

    return enc_theta;
}

float mock_encoder_read_theta(void)
{
    return enc_theta;
}

void mock_encoder_set_omega(float omega_rad_per_sec)
{
    enc_omega = omega_rad_per_sec;
}


/* ================================================================
 * PWM TIMER
 * ================================================================ */

void mock_timer_init(uint32_t ARR, float Vbus)
{
    tim_ARR  = ARR;
    tim_Vbus = Vbus;
    tim_CCR1 = 0;
    tim_CCR2 = 0;
    tim_CCR3 = 0;
}

void mock_pwm_write(uint32_t CCR1, uint32_t CCR2, uint32_t CCR3)
{
    /* Clamp to valid range [0, ARR] */
    tim_CCR1 = (CCR1 > tim_ARR) ? tim_ARR : CCR1;
    tim_CCR2 = (CCR2 > tim_ARR) ? tim_ARR : CCR2;
    tim_CCR3 = (CCR3 > tim_ARR) ? tim_ARR : CCR3;
}

void mock_pwm_read(uint32_t &CCR1, uint32_t &CCR2, uint32_t &CCR3)
{
    CCR1 = tim_CCR1;
    CCR2 = tim_CCR2;
    CCR3 = tim_CCR3;
}

uint32_t mock_timer_get_ARR(void)
{
    return tim_ARR;
}

float mock_timer_get_Vbus(void)
{
    return tim_Vbus;
}

void mock_pwm_print(void)
{
    float duty1 = (float)tim_CCR1 / (float)tim_ARR * 100.0f;
    float duty2 = (float)tim_CCR2 / (float)tim_ARR * 100.0f;
    float duty3 = (float)tim_CCR3 / (float)tim_ARR * 100.0f;

    printf("PWM | ARR=%u | CCR1=%u(%.1f%%) CCR2=%u(%.1f%%) CCR3=%u(%.1f%%)\n",
           tim_ARR,
           tim_CCR1, duty1,
           tim_CCR2, duty2,
           tim_CCR3, duty3);
}


/* ================================================================
 * GPIO
 * ================================================================ */

void mock_gpio_set_STO(bool active)
{
    gpio_STO_active = active;
}

void mock_gpio_set_fault(bool fault)
{
    gpio_fault = fault;
}

void mock_gpio_set_enable(bool enable)
{
    gpio_enable = enable;
}

bool mock_gpio_read_enable(void)
{
    return gpio_enable;
}

bool mock_gpio_read_STO(void)
{
    return gpio_STO_active;
}

void mock_gpio_print(void)
{
    printf("GPIO | STO=%s | FAULT=%s | ENABLE=%s\n",
           gpio_STO_active ? "ACTIVE" : "clear",
           gpio_fault      ? "YES"    : "no",
           gpio_enable     ? "YES"    : "no");
}
