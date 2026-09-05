#ifndef MOCK_HAL_H
#define MOCK_HAL_H

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/**
 * Mock HAL — Hardware Abstraction Layer Simulation
 * -------------------------------------------------
 * Simulates STM32G4 peripherals for algorithm testing.
 * No real hardware needed — runs on any x86 machine.
 *
 * Simulated peripherals:
 *     ADC     — phase current sensing (Ia, Ib)
 *     Encoder — rotor position (theta)
 *     PWM     — TIM1 centre aligned, CH1/2/3 and CH1N/2N/3N
 *     GPIO    — STO, fault, enable pins
 *
 * No dynamic memory. No exceptions. float and uint32_t only.
 * All state is static — exists for lifetime of program.
 */


/* ================================================================
 * CONSTANTS
 * ================================================================ */

#define MOCK_TWO_PI     6.28318530718f      /* 2 * pi                        */
#define MOCK_TWO_PI_3   2.09439510239f      /* 2 * pi / 3 — 120 degrees      */
#define MOCK_FOUR_PI_3  4.18879020479f      /* 4 * pi / 3 — 240 degrees      */


/* ================================================================
 * ADC — CURRENT SENSING
 * ================================================================
 *
 * Real hardware:
 *     Shunt resistors in phase legs
 *     ADC triggered at PWM centre (counter peak in centre aligned mode)
 *     DMA copies result to RAM
 *     ISR reads from RAM
 *
 * Mock:
 *     Returns sine wave at given theta
 *     Represents balanced 3 phase motor current
 */

/**
 * mock_adc_init
 * -------------
 * Set peak phase current amplitude.
 * Call once at startup.
 *
 * @param peak_amps  Peak phase current in amps (e.g. 10.0 for 10A motor)
 */
void mock_adc_init(float peak_amps);

/**
 * mock_adc_read_Ia
 * ----------------
 * Returns phase A current at given rotor angle.
 * Ia = peak * sin(theta)
 *
 * @param theta  Rotor electrical angle in radians
 * @return       Phase A current in amps
 */
float mock_adc_read_Ia(float theta);

/**
 * mock_adc_read_Ib
 * ----------------
 * Returns phase B current at given rotor angle.
 * Ib = peak * sin(theta - 2pi/3)
 * 120 degrees behind phase A.
 *
 * @param theta  Rotor electrical angle in radians
 * @return       Phase B current in amps
 */
float mock_adc_read_Ib(float theta);


/* ================================================================
 * ENCODER — ROTOR POSITION
 * ================================================================
 *
 * Real hardware:
 *     Incremental encoder — A/B/Z pulses
 *     TIM3 in encoder mode counts pulses
 *     theta = (count / pulses_per_rev) * 2 * pi
 *     Z pulse resets count every revolution
 *
 * Mock:
 *     theta advances by omega * dt each step
 *     Wraps at 2*pi back to 0
 */

/**
 * mock_encoder_init
 * -----------------
 * Set initial motor speed and starting angle.
 * Call once at startup.
 *
 * @param omega_rad_per_sec  Motor speed in rad/s
 *                           e.g. 50Hz motor = 2*pi*50 = 314.16 rad/s
 * @param theta_start        Starting angle in radians (usually 0.0)
 */
void mock_encoder_init(float omega_rad_per_sec, float theta_start);

/**
 * mock_encoder_step
 * -----------------
 * Advance theta by one time step.
 * Call once per FOC cycle — same rate as ADC reading.
 * Wraps theta to [0, 2*pi].
 *
 * @param dt  Time step in seconds (e.g. 0.0001 for 100us)
 * @return    Updated theta in radians
 */
float mock_encoder_step(float dt);

/**
 * mock_encoder_read_theta
 * -----------------------
 * Read current theta without advancing.
 * Use when same theta needed multiple times in one cycle.
 *
 * @return  Current theta in radians
 */
float mock_encoder_read_theta(void);

/**
 * mock_encoder_set_omega
 * ----------------------
 * Change motor speed during simulation.
 * Simulates operator changing speed reference.
 *
 * @param omega_rad_per_sec  New motor speed in rad/s
 */
void mock_encoder_set_omega(float omega_rad_per_sec);


/* ================================================================
 * PWM TIMER — TIM1 CENTRE ALIGNED
 * ================================================================
 *
 * Real hardware:
 *     TIM1 advanced timer — STM32G4
 *     Centre aligned mode 1 — counter counts up then down
 *     ARR = 8500 for 10kHz at 170MHz
 *         170MHz / (2 * 8500) = 10kHz
 *     6 output channels:
 *         CH1  / CH1N — Phase A high / low side
 *         CH2  / CH2N — Phase B high / low side
 *         CH3  / CH3N — Phase C high / low side
 *     CHxN is complement of CHx with dead time inserted
 *     Dead time set in BDTR register
 *     CCR value sets duty cycle:
 *         CCR = 0    → 0%   duty
 *         CCR = ARR  → 100% duty
 *         CCR = ARR/2 → 50% duty
 *
 * Mock:
 *     Stores CCR1, CCR2, CCR3 as uint32_t variables
 *     CHxN behaviour implied — not separately stored
 *     Dead time not simulated — algorithm level only
 */

/**
 * mock_timer_init
 * ---------------
 * Set ARR and DC bus voltage.
 * Call once at startup.
 *
 * @param ARR   Auto reload register value
 *              Sets PWM period and maximum CCR value
 *              8500 = 10kHz at 170MHz centre aligned
 * @param Vbus  DC bus voltage in volts (e.g. 600.0 for 400V motor drive)
 */
void mock_timer_init(uint32_t ARR, float Vbus);

/**
 * mock_pwm_write
 * --------------
 * Write CCR values — output of SVPWM.
 * In real hardware these write directly to TIM1->CCR1/2/3.
 * Values clamped to [0, ARR].
 *
 * @param CCR1  Phase A compare register value
 * @param CCR2  Phase B compare register value
 * @param CCR3  Phase C compare register value
 */
void mock_pwm_write(uint32_t CCR1, uint32_t CCR2, uint32_t CCR3);

/**
 * mock_pwm_read
 * -------------
 * Read back stored CCR values for verification.
 *
 * @param CCR1  Output — Phase A compare register value
 * @param CCR2  Output — Phase B compare register value
 * @param CCR3  Output — Phase C compare register value
 */
void mock_pwm_read(uint32_t &CCR1, uint32_t &CCR2, uint32_t &CCR3);

/**
 * mock_timer_get_ARR
 * ------------------
 * SVPWM needs ARR to normalise duty cycle calculation.
 *
 * @return  Current ARR value
 */
uint32_t mock_timer_get_ARR(void);

/**
 * mock_timer_get_Vbus
 * -------------------
 * SVPWM needs Vbus to normalise voltage vectors.
 *
 * @return  DC bus voltage in volts
 */
float mock_timer_get_Vbus(void);

/**
 * mock_pwm_print
 * --------------
 * Print current CCR values and duty cycles to console.
 * Duty cycle = CCR / ARR * 100%
 */
void mock_pwm_print(void);


/* ================================================================
 * GPIO — FAULT AND ENABLE PINS
 * ================================================================
 *
 * Real hardware:
 *     STO  — Safe Torque Off — hardware input, disables gate driver
 *             Motor coasts — no torque, no braking
 *             Requires mechanical brake for suspended loads
 *     FAULT — output to PLC — drive has faulted
 *     ENABLE — input from PLC — permission to run
 *
 * Mock:
 *     Simple bool variables
 *     foc_controller checks these before running
 */

/**
 * mock_gpio_set_STO
 * -----------------
 * @param active  true = STO active = gate driver disabled = motor coasts
 */
void mock_gpio_set_STO(bool active);

/**
 * mock_gpio_set_fault
 * -------------------
 * @param fault  true = drive has faulted, output to PLC
 */
void mock_gpio_set_fault(bool fault);

/**
 * mock_gpio_set_enable
 * --------------------
 * @param enable  true = PLC has given permission to run
 */
void mock_gpio_set_enable(bool enable);

/**
 * mock_gpio_read_enable
 * ---------------------
 * @return  true = drive is enabled by PLC
 */
bool mock_gpio_read_enable(void);

/**
 * mock_gpio_read_STO
 * ------------------
 * @return  true = STO is active = must not run
 */
bool mock_gpio_read_STO(void);

/**
 * mock_gpio_print
 * ---------------
 * Print all GPIO pin states to console.
 */
void mock_gpio_print(void);


#endif /* MOCK_HAL_H */
