#ifndef NVM_H
#define NVM_H

#include "drive_state.h"
#include "fault_manager.h"
#include <stdbool.h>

/**
 * NVM — Non Volatile Memory (Mock EEPROM)
 * ----------------------------------------
 * In real hardware: STM32G4 internal EEPROM or external I2C EEPROM
 * In simulation: static arrays in RAM — survives within one run
 *
 * Stores:
 *     Drive parameters — PI gains, speed limits, motor data
 *     Fault history    — last 50 fault entries
 *     Running hours    — total operating time
 *     Checksum         — detects corrupted data
 *
 * No dynamic memory. No exceptions.
 */


/* ================================================================
 * NVM FUNCTIONS
 * ================================================================ */

/**
 * nvm_init
 * --------
 * Initialise NVM mock.
 * Checks if valid data exists.
 * If not — writes defaults.
 * Call once at startup before reading.
 */
void nvm_init(void);

/**
 * nvm_save_params
 * ---------------
 * Save drive parameters to NVM.
 * Called when engineer changes parameters.
 * Called on controlled shutdown.
 *
 * @param params  Parameters to save
 * @return        true if saved successfully
 */
bool nvm_save_params(const DriveParams* params);

/**
 * nvm_load_params
 * ---------------
 * Load drive parameters from NVM.
 * Called at startup.
 * If NVM corrupt — loads defaults.
 *
 * @param params  Where to load parameters into
 * @return        true if loaded successfully, false if defaults used
 */
bool nvm_load_params(DriveParams* params);

/**
 * nvm_save_fault_history
 * ----------------------
 * Save fault buffer to NVM.
 * Called after every new fault logged.
 *
 * @param entries  Array of fault entries
 * @param count    Number of entries to save
 * @return         true if saved successfully
 */
bool nvm_save_fault_history(const FaultEntry* entries, uint8_t count);

/**
 * nvm_load_fault_history
 * ----------------------
 * Load fault history from NVM.
 * Called at startup to restore previous fault log.
 *
 * @param entries    Array to load faults into
 * @param max_count  Maximum entries to load
 * @return           Number of entries loaded
 */
uint8_t nvm_load_fault_history(FaultEntry* entries, uint8_t max_count);

/**
 * nvm_save_running_hours
 * ----------------------
 * Save total running hours counter.
 * Called periodically every hour of operation.
 *
 * @param hours  Total running hours to save
 */
void nvm_save_running_hours(uint32_t hours);

/**
 * nvm_load_running_hours
 * ----------------------
 * Load running hours from NVM.
 *
 * @return  Total running hours
 */
uint32_t nvm_load_running_hours(void);

/**
 * nvm_clear_all
 * -------------
 * Erase all NVM data and restore defaults.
 * Use with caution — loses all fault history and parameters.
 * Called by engineer during full drive reset.
 */
void nvm_clear_all(void);

/**
 * nvm_print_status
 * ----------------
 * Print NVM contents to console for debugging.
 */
void nvm_print_status(void);


/* ================================================================
 * DEFAULT PARAMETERS
 * Used when NVM is empty or corrupt
 * ================================================================ */

static const DriveParams NVM_DEFAULT_PARAMS = {
    /* Motor parameters */
    .Rs              = 1.0f,
    .Ls              = 0.005f,
    .rated_current   = 10.0f,
    .rated_speed_rpm = 1500.0f,

    /* PI gains */
    .Kp_id           = 1.0f,
    .Ki_id           = 10.0f,
    .Kp_iq           = 1.0f,
    .Ki_iq           = 10.0f,

    /* Speed limits */
    .speed_min_rpm   = 0.0f,
    .speed_max_rpm   = 1500.0f,
    .accel_rate      = 100.0f,
    .decel_rate      = 100.0f,

    /* Protection thresholds */
    .overcurrent_limit  = 15.0f,
    .overvoltage_limit  = 700.0f,
    .undervoltage_limit = 400.0f,
    .overtemp_limit     = 85.0f,
    .temp_alarm_limit   = 75.0f
};


#endif /* NVM_H */