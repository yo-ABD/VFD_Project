#include "nvm.h"
#include "fault_manager.h"
#include <stdio.h>
#include <string.h>

/**
 * Mock NVM — simulates EEPROM in RAM
 * In real hardware: HAL_FLASH_Program or I2C EEPROM write
 * Here: static arrays that persist within one program run
 */

/* ================================================================
 * MOCK NVM STORAGE
 * ================================================================ */

static DriveParams  s_nvm_params;
static FaultEntry   s_nvm_faults[FAULT_BUFFER_SIZE];
static uint8_t      s_nvm_fault_count  = 0;
static uint32_t     s_nvm_running_hours = 0;
static bool         s_nvm_params_valid  = false;
static bool         s_nvm_faults_valid  = false;


/* ================================================================
 * nvm_init
 * ================================================================ */

void nvm_init(void)
{
    /* In real hardware: check flash checksum here */
    /* In mock: just initialise flags */
    printf("[NVM] Initialised\n");
}


/* ================================================================
 * nvm_save_params
 * ================================================================ */

bool nvm_save_params(const DriveParams* params)
{
    memcpy(&s_nvm_params, params, sizeof(DriveParams));
    s_nvm_params_valid = true;
    printf("[NVM] Parameters saved\n");
    return true;
}


/* ================================================================
 * nvm_load_params
 * ================================================================ */

bool nvm_load_params(DriveParams* params)
{
    if (!s_nvm_params_valid)
    {
        /* No valid data — load defaults */
        memcpy(params, &NVM_DEFAULT_PARAMS, sizeof(DriveParams));
        return false;
    }

    memcpy(params, &s_nvm_params, sizeof(DriveParams));
    return true;
}


/* ================================================================
 * nvm_save_fault_history
 * ================================================================ */

bool nvm_save_fault_history(const FaultEntry* entries, uint8_t count)
{
    if (count > FAULT_BUFFER_SIZE)
    {
        count = FAULT_BUFFER_SIZE;
    }

    memcpy(s_nvm_faults, entries, count * sizeof(FaultEntry));
    s_nvm_fault_count = count;
    s_nvm_faults_valid = true;
    return true;
}


/* ================================================================
 * nvm_load_fault_history
 * ================================================================ */

uint8_t nvm_load_fault_history(FaultEntry* entries, uint8_t max_count)
{
    if (!s_nvm_faults_valid || s_nvm_fault_count == 0)
    {
        return 0;
    }

    uint8_t count = s_nvm_fault_count;
    if (count > max_count)
    {
        count = max_count;
    }

    memcpy(entries, s_nvm_faults, count * sizeof(FaultEntry));
    return count;
}


/* ================================================================
 * nvm_save_running_hours
 * ================================================================ */

void nvm_save_running_hours(uint32_t hours)
{
    s_nvm_running_hours = hours;
}


/* ================================================================
 * nvm_load_running_hours
 * ================================================================ */

uint32_t nvm_load_running_hours(void)
{
    return s_nvm_running_hours;
}


/* ================================================================
 * nvm_clear_all
 * ================================================================ */

void nvm_clear_all(void)
{
    memset(&s_nvm_params, 0, sizeof(DriveParams));
    memset(s_nvm_faults, 0, sizeof(s_nvm_faults));
    s_nvm_fault_count   = 0;
    s_nvm_running_hours = 0;
    s_nvm_params_valid  = false;
    s_nvm_faults_valid  = false;
    printf("[NVM] All data cleared\n");
}


/* ================================================================
 * nvm_print_status
 * ================================================================ */

void nvm_print_status(void)
{
    printf("\n[NVM] Status:\n");
    printf("  Parameters valid: %s\n", s_nvm_params_valid ? "YES" : "NO");
    printf("  Fault entries:    %u\n", s_nvm_fault_count);
    printf("  Running hours:    %u\n", s_nvm_running_hours);

    if (s_nvm_params_valid)
    {
        printf("  Rs=%.2f  Ls=%.4f  Kp_iq=%.2f  Ki_iq=%.2f\n",
               s_nvm_params.Rs,
               s_nvm_params.Ls,
               s_nvm_params.Kp_iq,
               s_nvm_params.Ki_iq);
        printf("  Speed max=%.0f RPM  Overcurrent limit=%.1fA\n",
               s_nvm_params.speed_max_rpm,
               s_nvm_params.overcurrent_limit);
    }
}