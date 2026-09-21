#include "fault_manager.h"
#include "nvm.h"
#include <stdio.h>
#include <string.h>

/* ================================================================
 * STATIC STATE — circular buffer
 * ================================================================ */

static FaultEntry s_buffer[FAULT_BUFFER_SIZE];
static uint8_t    s_head  = 0;    /* next write position           */
static uint8_t    s_count = 0;    /* number of entries stored      */


/* ================================================================
 * fault_init
 * ================================================================ */

void fault_init(void)
{
    memset(s_buffer, 0, sizeof(s_buffer));
    s_head  = 0;
    s_count = 0;

    /* Load fault history from NVM */
    uint8_t loaded = nvm_load_fault_history(s_buffer, FAULT_BUFFER_SIZE);
    if (loaded > 0)
    {
        s_count = loaded;
        s_head  = loaded % FAULT_BUFFER_SIZE;
        printf("[FAULT] Loaded %u fault entries from NVM\n", loaded);
    }
    else
    {
        printf("[FAULT] No fault history in NVM\n");
    }
}


/* ================================================================
 * fault_log
 * ================================================================ */

void fault_log(FaultCode code,
               float Vbus,
               float Ia,
               float Id,
               float Iq,
               float speed_rpm)
{
    /* Build fault entry */
    FaultEntry entry;
    entry.timestamp_ms   = drive_get_uptime_ms();
    entry.code           = code;
    entry.state_at_fault = drive_get_state();
    entry.Vbus           = Vbus;
    entry.Ia             = Ia;
    entry.Id             = Id;
    entry.Iq             = Iq;
    entry.speed_rpm      = speed_rpm;

    /* Write to circular buffer */
    s_buffer[s_head] = entry;

    /* Advance head — wrap at buffer size */
    s_head = (s_head + 1) % FAULT_BUFFER_SIZE;

    /* Increment count up to max */
    if (s_count < FAULT_BUFFER_SIZE)
    {
        s_count++;
    }

    /* Print to console */
    printf("[FAULT] Logged: %s at %ums — Vbus=%.1fV Ia=%.2fA Iq=%.2fA\n",
           drive_fault_name(code),
           entry.timestamp_ms,
           Vbus, Ia, Iq);

    /* Save to NVM */
    nvm_save_fault_history(s_buffer, s_count);
}


/* ================================================================
 * fault_get
 * Index 0 = most recent fault
 * ================================================================ */

FaultEntry fault_get(uint8_t index)
{
    FaultEntry empty;
    memset(&empty, 0, sizeof(FaultEntry));

    if (index >= s_count)
    {
        return empty;
    }

    /* Most recent is at (head - 1), going backwards */
    int pos = (int)s_head - 1 - (int)index;
    if (pos < 0)
    {
        pos += FAULT_BUFFER_SIZE;
    }

    return s_buffer[pos];
}


/* ================================================================
 * fault_get_count
 * ================================================================ */

uint8_t fault_get_count(void)
{
    return s_count;
}


/* ================================================================
 * fault_get_last_code
 * ================================================================ */

FaultCode fault_get_last_code(void)
{
    if (s_count == 0)
    {
        return FAULT_NONE;
    }

    return fault_get(0).code;
}


/* ================================================================
 * fault_clear_all
 * ================================================================ */

void fault_clear_all(void)
{
    memset(s_buffer, 0, sizeof(s_buffer));
    s_head  = 0;
    s_count = 0;
    nvm_save_fault_history(s_buffer, 0);
    printf("[FAULT] All faults cleared\n");
}


/* ================================================================
 * fault_get_severity
 * ================================================================ */

FaultSeverity fault_get_severity(FaultCode code)
{
    switch (code)
    {
        /* Alarms — motor keeps running */
        case ALARM_TEMP_WARNING:
        case ALARM_CURRENT_HIGH:
        case ALARM_VBUS_HIGH:
            return SEVERITY_ALARM;

        /* Recoverable faults — operator can reset */
        case FAULT_OVERCURRENT:
        case FAULT_OVERVOLTAGE:
        case FAULT_UNDERVOLTAGE:
        case FAULT_OVERTEMPERATURE:
        case FAULT_POWER_LOSS:
            return SEVERITY_RECOVERABLE;

        /* Critical faults — engineer required */
        case FAULT_ENCODER_LOSS:
        case FAULT_IGBT_DESAT:
        case FAULT_EARTH:
            return SEVERITY_LOCKOUT;

        default:
            return SEVERITY_NONE;
    }
}


/* ================================================================
 * fault_print_entry
 * ================================================================ */

void fault_print_entry(const FaultEntry* entry)
{
    printf("  [%6ums] %-22s state=%-18s Vbus=%5.1fV Ia=%6.2fA Id=%6.2fA Iq=%6.2fA speed=%6.1fRPM\n",
           entry->timestamp_ms,
           drive_fault_name(entry->code),
           drive_state_name(entry->state_at_fault),
           entry->Vbus,
           entry->Ia,
           entry->Id,
           entry->Iq,
           entry->speed_rpm);
}


/* ================================================================
 * fault_print_all
 * ================================================================ */

void fault_print_all(void)
{
    printf("\n[FAULT] Fault History — %u entries\n", s_count);
    printf("  %-8s %-22s %-18s %-8s %-8s %-8s %-8s %-10s\n",
           "Time(ms)", "Fault", "State", "Vbus", "Ia", "Id", "Iq", "Speed");
    printf("  %s\n", "------------------------------------------------------------");

    if (s_count == 0)
    {
        printf("  No faults logged\n");
        return;
    }

    for (uint8_t i = 0; i < s_count; i++)
    {
        FaultEntry entry = fault_get(i);
        fault_print_entry(&entry);
    }
}