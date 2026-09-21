#ifndef FAULT_MANAGER_H
#define FAULT_MANAGER_H

#include "drive_state.h"

/**
 * Fault Manager
 * -------------
 * Circular buffer of fault entries.
 * Static allocation — no malloc.
 * 50 entries maximum.
 * Oldest overwritten when full.
 *
 * Also manages NVM save and load of fault history.
 */


/* ================================================================
 * CONSTANTS
 * ================================================================ */

#define FAULT_BUFFER_SIZE   50      /* maximum fault entries stored  */


/* ================================================================
 * FAULT MANAGER FUNCTIONS
 * ================================================================ */

/**
 * fault_init
 * ----------
 * Initialise fault manager.
 * Clears buffer.
 * Loads fault history from NVM.
 * Call once at startup.
 */
void fault_init(void);

/**
 * fault_log
 * ---------
 * Log a fault or alarm event.
 * Stores in circular buffer.
 * Oldest entry overwritten when buffer full.
 * Automatically saves to NVM.
 *
 * @param code      Fault or alarm code
 * @param Vbus      DC bus voltage at time of fault
 * @param Ia        Phase A current at time of fault
 * @param Id        Flux current at time of fault
 * @param Iq        Torque current at time of fault
 * @param speed_rpm Motor speed at time of fault
 */
void fault_log(FaultCode code,
               float Vbus,
               float Ia,
               float Id,
               float Iq,
               float speed_rpm);

/**
 * fault_get
 * ---------
 * Read a fault entry from buffer.
 * Index 0 = most recent fault.
 * Index 1 = second most recent.
 * ...and so on.
 *
 * @param index  Entry index (0 = newest)
 * @return       FaultEntry struct
 */
FaultEntry fault_get(uint8_t index);

/**
 * fault_get_count
 * ---------------
 * Returns number of faults currently stored.
 * Maximum FAULT_BUFFER_SIZE.
 *
 * @return  Number of stored fault entries
 */
uint8_t fault_get_count(void);

/**
 * fault_get_last_code
 * -------------------
 * Returns fault code of most recent fault.
 * Returns FAULT_NONE if no faults logged.
 */
FaultCode fault_get_last_code(void);

/**
 * fault_clear_all
 * ---------------
 * Clears entire fault buffer.
 * Call when engineer acknowledges all faults.
 * Also clears NVM fault history.
 */
void fault_clear_all(void);

/**
 * fault_get_severity
 * ------------------
 * Returns severity level of a fault code.
 * Used by state machine to determine which state to enter.
 *
 * @param code  Fault code to check
 * @return      Severity level
 */
FaultSeverity fault_get_severity(FaultCode code);

/**
 * fault_print_all
 * ---------------
 * Print all stored faults to console.
 * For debugging and commissioning.
 */
void fault_print_all(void);

/**
 * fault_print_entry
 * -----------------
 * Print one fault entry to console.
 *
 * @param entry  Fault entry to print
 */
void fault_print_entry(const FaultEntry* entry);


#endif /* FAULT_MANAGER_H */