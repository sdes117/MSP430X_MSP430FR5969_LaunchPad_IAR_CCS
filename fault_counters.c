/*
 * fault_counters.c
 *
 * MSP-local fault counter state and management.
 * See fault_counters.h for full documentation.
 */

#include "fault_counters.h"
#include "rp_regmap.h"

/* ======================================================================
 * State — FRAM-persistent across warm resets where possible.
 * For CCS: use #pragma NOINIT or #pragma PERSISTENT as appropriate.
 * These are declared volatile to prevent the compiler optimising away
 * reads that check them from ISR/task context.
 * ====================================================================== */

volatile uint16_t g_msp_fault_bitmap              = 0u;
volatile uint16_t g_msp_fault_latch               = 0u;
volatile uint8_t  g_msp_fault_counts[MSP_FCNT_COUNT] = {0u};
volatile uint32_t g_fault_decay_timer_s           = 0u;

/* ======================================================================
 * Internal helpers
 * ====================================================================== */

/* Map fault bitmap bit → counter index. Returns MSP_FCNT_COUNT if no match. */
static uint8_t prvBitToCounterIdx(uint16_t fault_bit)
{
    switch (fault_bit)
    {
    case MSP_FAULT_PWR_UV_BATT:   return MSP_FCNT_PWR_UV_BATT;
    case MSP_FAULT_PWR_UV_LOAD:   return MSP_FCNT_PWR_UV_LOAD;
    case MSP_FAULT_PWR_OC_MCU:    return MSP_FCNT_PWR_OC_MCU;
    case MSP_FAULT_WDT_MSP_MISS:  return MSP_FCNT_WDT_MSP_MISS;
    case MSP_FAULT_WDT_EXT_TRIP:  return MSP_FCNT_WDT_EXT_TRIP;
    case MSP_FAULT_WDT_RP_MISS:   return MSP_FCNT_WDT_RP_MISS;
    case MSP_FAULT_THERM_OVERTEMP:return MSP_FCNT_THERM_OVERTEMP;
    case MSP_FAULT_THERM_BATT_LOW:return MSP_FCNT_THERM_BATT_LOW;
    case MSP_FAULT_I2C_LINK_ERR:      return MSP_FCNT_I2C_LINK_ERR;
    case MSP_FAULT_MEM_CRC:           return MSP_FCNT_MEM_CRC;
    case MSP_FAULT_RP_TX_SHUTDOWN:    return MSP_FCNT_RP_TX_SHUTDOWN;
    default:                           return MSP_FCNT_COUNT;
    }
}

/* ======================================================================
 * Public API
 * ====================================================================== */

void fault_set(uint16_t fault_bit, uint8_t counter_idx)
{
    g_msp_fault_bitmap |= fault_bit;
    g_msp_fault_latch  |= fault_bit;

    if (counter_idx < MSP_FCNT_COUNT)
    {
        FAULT_CNT_INC(g_msp_fault_counts[counter_idx]);
    }
}

void fault_clear(uint16_t fault_bit)
{
    g_msp_fault_bitmap &= (uint16_t)~fault_bit;
}

void fault_clear_latch(uint16_t fault_bit)
{
    g_msp_fault_latch &= (uint16_t)~fault_bit;
}

void fault_tick_1hz(void)
{
    g_fault_decay_timer_s++;

    if (g_fault_decay_timer_s >= FAULT_DECAY_INTERVAL_S)
    {
        g_fault_decay_timer_s = 0u;

        /* Decay count-type faults by 1 per hour.
         * Latching faults (WDT_EXT_TRIP, PWR_UV_BATT, etc.) are not
         * decayed here — they are cleared explicitly by policy. */
        FAULT_CNT_DECAY(g_msp_fault_counts[MSP_FCNT_PWR_UV_LOAD]);
        FAULT_CNT_DECAY(g_msp_fault_counts[MSP_FCNT_WDT_MSP_MISS]);
        FAULT_CNT_DECAY(g_msp_fault_counts[MSP_FCNT_I2C_LINK_ERR]);
        /* F_THERM_BATT_LOW decays too (count-type) */
        FAULT_CNT_DECAY(g_msp_fault_counts[MSP_FCNT_THERM_BATT_LOW]);
    }
}

void fault_pack_status(uint8_t msp_mode,
                       uint8_t wdt_ok,
                       uint8_t batt_ok,
                       uint8_t rail_3v3_ok,
                       uint8_t *status0_out,
                       uint8_t *status1_out)
{
    uint8_t  s0 = 0u;
    uint8_t  s1 = 0u;
    uint16_t bm = g_msp_fault_bitmap;
    uint8_t  i;

    /* STATUS0 layout (matches MSP_STATUS0_* defines in rp_regmap.h):
     * bits [2:0] = MSP mode
     * bit3       = external WDTs OK
     * bit4       = battery voltage OK
     * bit5       = 3V3 rail PG
     * bit7       = any P0 fault active */
    s0 = (uint8_t)(msp_mode & 0x07u);
    if (wdt_ok     != 0u) { s0 |= MSP_STATUS0_WDT_OK; }
    if (batt_ok    != 0u) { s0 |= MSP_STATUS0_BATT_OK; }
    if (rail_3v3_ok!= 0u) { s0 |= MSP_STATUS0_RAIL_3V3_OK; }

    /* P0 faults that trigger STATUS0 fault flag — must match P0_MASK in
     * mode_state_machine.c so ground telemetry reflects escalation state. */
    if (bm & (MSP_FAULT_PWR_UV_BATT    |
              MSP_FAULT_WDT_MSP_MISS   |
              MSP_FAULT_WDT_EXT_TRIP   |
              MSP_FAULT_WDT_RP_MISS    |
              MSP_FAULT_THERM_OVERTEMP |
              MSP_FAULT_RP_TX_SHUTDOWN))
    {
        s0 |= MSP_STATUS0_FAULT;
    }

    /* STATUS1 = count of active fault bits (population count, saturated) */
    for (i = 0u; i < 16u; i++)
    {
        if (bm & (uint16_t)(1u << i))
        {
            if (s1 < 0xFFu) { s1++; }
        }
    }

    *status0_out = s0;
    *status1_out = s1;
}
