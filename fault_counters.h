/*
 * fault_counters.h
 *
 * System fault counter and bitmap definitions for the MSP430FR supervisor.
 * Sourced from HIA_Task_Schedule_MSP_Master_RP_Slave_v4.xlsx,
 * "Fault Counters" sheet.
 *
 * Two fault domains:
 *
 *   MSP-local faults  — tracked in g_msp_fault_bitmap / g_msp_fault_counts[]
 *                        entirely within the supervisor. Includes EPS, thermal,
 *                        WDT and shared I2C link faults.
 *
 *   RP-reported faults — read from RP regmap FAULT_BITMAP_RP (0xF0-0xF1) and
 *                        the per-counter registers (0xF4-0xFC). Bit definitions
 *                        are in rp_regmap.h (RP_FAULT_* macros).
 *
 * Counter types:
 *   COUNT    — saturating uint8_t; decays over time; cleared by policy.
 *   LATCH    — sticky bit set on first occurrence; cleared only explicitly.
 *   STATE    — reflects a transient operating condition, not a "true" fault.
 *
 * Severity:
 *   P0 — safety-critical; immediate action required (escalation / SURVIVAL).
 *   P1 — high; may degrade operation; log and monitor.
 *   P2 — medium; retry / recover; log.
 *   P3 — low; statistics / link quality; log only.
 *   P4 — informational.
 */

#ifndef FAULT_COUNTERS_H_
#define FAULT_COUNTERS_H_

#include <stdint.h>

/* ======================================================================
 * MSP-local fault bitmap bits
 * (uint16_t g_msp_fault_bitmap — supervisor-internal, not sent over I2C
 *  directly; packed into MSP_STATUS0/1 summary for downlink)
 * ====================================================================== */

/* --- EPS / Power faults (MSP-owned) --- */

/* F_PWR_UV_BATT  — P0 latching+count
 * VBATT < Vcrit for >N consecutive samples.
 * Primary SURVIVAL trigger. Src: INA219.
 * Clear: VBATT > Vrec for 10 min + policy. */
#define MSP_FAULT_PWR_UV_BATT       (0x0001u)  /* bit0 */

/* F_PWR_UV_LOAD  — P1 count+time
 * Load rail undervoltage / brownout.
 * Src: eFuse PG/FLT + INA219s.
 * Decay: 1/hr; clear when stable for 5 min. */
#define MSP_FAULT_PWR_UV_LOAD       (0x0002u)  /* bit1 */

/* F_PWR_OC_MCU  — P1 latching+count
 * Overcurrent/fault on EPS→MCU eFuse path (single line only).
 * Src: PG and FLT signals.
 * Clear: after power-cycle + stable rail. */
#define MSP_FAULT_PWR_OC_MCU        (0x0004u)  /* bit2 */

/* --- Supervisor / WDT faults (MSP-owned) --- */

/* F_WDT_MSP_MISS  — P0 count
 * MSP internal WDT nearly expired / missed tick (supervisor starvation).
 * Clear: on next good tick; always log. */
#define MSP_FAULT_WDT_MSP_MISS      (0x0008u)  /* bit3 */

/* F_WDT_EXT_TRIP  — P0 latching+count
 * External watchdog (TPS3435) reset asserted.
 * Src: SYSRSTIV reset-reason register on boot.
 * Clear: only after post-reset logging + optional ground ack. */
#define MSP_FAULT_WDT_EXT_TRIP      (0x0010u)  /* bit4 */

/* F_WDT_RP_MISS  — P0 count+time
 * RP heartbeat GPIO (WDT_RP2MSP1/2) missing for > timeout.
 * Also covers I2C keepalive timeout.
 * Triggers reset / power-cycle escalation ladder.
 * Clear: after RP healthy for 2 min. */
#define MSP_FAULT_WDT_RP_MISS       (0x0020u)  /* bit5 */

/* --- Thermal faults (MSP-owned) --- */

/* F_THERM_OVERTEMP  — P0 latching+count
 * Battery or PCB temp > Tmax.
 * May force SAFE or SURVIVAL.
 * Clear: when < Tmax-delta for 10 min + policy. */
#define MSP_FAULT_THERM_OVERTEMP    (0x0040u)  /* bit6 */

/* F_THERM_BATT_LOW  — P2 count
 * Battery temp below heater-on threshold.
 * Drives heater control policy.
 * Clear: when above off-threshold with hysteresis. */
#define MSP_FAULT_THERM_BATT_LOW    (0x0080u)  /* bit7 */

/* --- Link faults (MSP-observed, shared with RP) --- */

/* F_I2C_LINK_ERR  — P1 count
 * I2C regmap errors, repeated NACKs, SDA/SCL timeouts, bus stuck.
 * Decay: 1/hr; clear when stable for 10 min. */
#define MSP_FAULT_I2C_LINK_ERR      (0x0100u)  /* bit8 */

/* F_MEM_CRC  — P1 latching+count
 * Computed CRC32 over code FRAM differs from baseline established at boot.
 * Possible bit-flip or unverified firmware write.
 * Clear: after explicit re-baseline (firmware update commit). */
#define MSP_FAULT_MEM_CRC           (0x0200u)  /* bit9 */

/* F_RP_TX_SHUTDOWN  — P0 latching
 * RP permanent TX shutdown flag (RP_FAULT_TX_SHUTDOWN_PERM) asserted.
 * Regulatory P0 event: MSP must escalate to SAFE and log.
 * Clear: explicit ground ack only. */
#define MSP_FAULT_RP_TX_SHUTDOWN    (0x0400u)  /* bit10 */

/* bits [15:11] — spare, reserved for future MSP-local faults */

/* ======================================================================
 * MSP fault counter indices
 * g_msp_fault_counts[MSP_FCNT_*] — saturating uint8_t each
 * ====================================================================== */
#define MSP_FCNT_PWR_UV_BATT        (0u)
#define MSP_FCNT_PWR_UV_LOAD        (1u)
#define MSP_FCNT_PWR_OC_MCU         (2u)
#define MSP_FCNT_WDT_MSP_MISS       (3u)
#define MSP_FCNT_WDT_EXT_TRIP       (4u)
#define MSP_FCNT_WDT_RP_MISS        (5u)
#define MSP_FCNT_THERM_OVERTEMP     (6u)
#define MSP_FCNT_THERM_BATT_LOW     (7u)
#define MSP_FCNT_I2C_LINK_ERR       (8u)
#define MSP_FCNT_MEM_CRC            (9u)
#define MSP_FCNT_RP_TX_SHUTDOWN     (10u)
#define MSP_FCNT_COUNT              (11u) /* total number of MSP counters */

/* ======================================================================
 * Severity thresholds and decay constants
 * ====================================================================== */

/* Minimum VBATT samples below Vcrit before latching F_PWR_UV_BATT */
#define FAULT_UV_BATT_CONSEC_SAMPLES    (3u)

/* VBATT recovery time before clearing F_PWR_UV_BATT (seconds) */
#define FAULT_UV_BATT_RECOVERY_S        (600u)  /* 10 min */

/* Load rail stable time before clearing F_PWR_UV_LOAD (seconds) */
#define FAULT_UV_LOAD_STABLE_S          (300u)  /* 5 min */

/* I2C stable time before clearing F_I2C_LINK_ERR (seconds) */
#define FAULT_I2C_STABLE_S              (600u)  /* 10 min */

/* RP healthy time before clearing F_WDT_RP_MISS (seconds) */
#define FAULT_WDT_RP_RECOVERY_S         (120u)  /* 2 min */

/* Overtemp recovery time (below Tmax-delta) before clearing (seconds) */
#define FAULT_OVERTEMP_RECOVERY_S       (600u)  /* 10 min */

/* Hourly decay tick: decrement count-type faults by 1 every N seconds */
#define FAULT_DECAY_INTERVAL_S          (3600u) /* 1 hour */

/* ======================================================================
 * Fault counter saturation limit
 * ====================================================================== */
#define FAULT_CNT_MAX   (0xFFu)

/* Saturating increment helper */
#define FAULT_CNT_INC(cnt) \
    do { if ((cnt) < FAULT_CNT_MAX) { ++(cnt); } } while (0)

/* Saturating decrement (decay) helper */
#define FAULT_CNT_DECAY(cnt) \
    do { if ((cnt) > 0u) { --(cnt); } } while (0)

/* ======================================================================
 * MSP-local fault state (extern declarations — defined in fault_counters.c)
 * ====================================================================== */

/* Active fault bitmap — bit set = fault currently active */
extern volatile uint16_t g_msp_fault_bitmap;

/* Latched fault bitmap — bit set = fault has occurred since last clear */
extern volatile uint16_t g_msp_fault_latch;

/* Per-fault saturating counters */
extern volatile uint8_t  g_msp_fault_counts[MSP_FCNT_COUNT];

/* Seconds since last hourly decay tick */
extern volatile uint32_t g_fault_decay_timer_s;

/* ======================================================================
 * Fault management API
 * ====================================================================== */

/*
 * Set a fault bit in both the active bitmap and the latch bitmap,
 * and increment its counter.
 */
void fault_set(uint16_t fault_bit, uint8_t counter_idx);

/*
 * Clear a fault bit from the active bitmap only (latch remains set).
 * Does not touch the counter.
 */
void fault_clear(uint16_t fault_bit);

/*
 * Explicitly clear the latch bit for a fault (e.g. after ground ack).
 */
void fault_clear_latch(uint16_t fault_bit);

/*
 * Called once per second from the clock task.
 * Applies hourly decay to count-type faults.
 */
void fault_tick_1hz(void);

/*
 * Pack active bitmap + latch into the two MSP_STATUS bytes for I2C downlink.
 * STATUS0 bits [2:0] = MSP mode; bit3 = WDT_OK; bit4 = BATT_OK;
 *                      bit5 = RAIL_3V3_OK; bit7 = any P0 fault active.
 * STATUS1            = count of active fault bits (saturated to 0xFF).
 */
void fault_pack_status(uint8_t msp_mode,
                       uint8_t wdt_ok,
                       uint8_t batt_ok,
                       uint8_t rail_3v3_ok,
                       uint8_t *status0_out,
                       uint8_t *status1_out);

#endif /* FAULT_COUNTERS_H_ */
