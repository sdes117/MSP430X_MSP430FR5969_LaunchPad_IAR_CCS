/*
 * main_blinky.c  —  MSP430FR5969 HIA supervisor (bring-up build)
 *
 * Tasks
 * -----
 *  CLK  (1 Hz)   : increments second counter, drives RP regmap (writes + reads)
 *  Rx   (1 Hz)   : receives tick, toggles LEDs, pulses external WDT lines,
 *                  runs LED blink-code for RP state indication
 *  INA  (2 Hz)   : reads INA219 @ 0x41 (3V3_MSP supply current)
 *
 * Register map
 * ------------
 *  MSP writes:  0x20–0x21 (status), 0x22–0x27 (contact ack stub),
 *               0x28 (mode cmd), 0x98–0x9A (voltage/current TLM),
 *               0xA5 (TLM mode), 0xAE–0xAF (TLM status mirrors)
 *  MSP reads:   0x00–0x03 (header magic), 0x10–0x17 (RP state/health),
 *               0x30–0x33 (RP request flags), 0xF0–0xF5 (fault bitmap)
 *  All stored in g_rp_reg_snapshot (FRAM) — inspect via JTAG after a run.
 *
 * Debug — no serial
 * ------------------
 *  1. FRAM snapshot  (g_rp_reg_snapshot) persists across resets.
 *     Inspect in CCS debugger: hdr_ok, rp_state, rp_uptime_s, fault_bitmap.
 *  2. LED blink code (LED0, P1.3) every 8 seconds:
 *       N blinks = rp_state + 1   (1=BOOT, 2=INIT, 3=NOMINAL, 4=SAFE, 5=FAULT)
 *       Rapid 10 Hz blink         = RP not responding (grace/reset active)
 *
 * Rail control
 * ------------
 *  Debugger-accessible globals g_rega_en / g_regb_en / g_efusea_en / g_efuseb_en
 *  control rail enable/shutdown pins. 1=enabled (default), 0=disabled.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "partest.h"
#include "driverlib.h"
#include <msp430.h>
#include <string.h>

#include "supervisor_i2c.h"
#include "rp_regmap.h"
#include "ina219.h"

/* ------------------------------------------------------------------
 * Task priorities
 * ------------------------------------------------------------------ */
#define PRIORITY_CLK    ( tskIDLE_PRIORITY + 3u )
#define PRIORITY_RX     ( tskIDLE_PRIORITY + 2u )
#define PRIORITY_INA    ( tskIDLE_PRIORITY + 1u )

/* ------------------------------------------------------------------
 * WDT pulse config (MSP external WDT chip heartbeats)
 * ------------------------------------------------------------------ */
#define WDT_LOW_CYCLES          ( 8U )
#define MSP_WDT_PERIOD_S      ( 2U )

/* ------------------------------------------------------------------
 * RP watchdog monitor
 * ------------------------------------------------------------------ */
#define RP_WDT_TIMEOUT_S        ( 6U )
#define RP_WDT_GRACE_S          ( 10U )
#define RESET_RP_PORT           GPIO_PORT_P2
#define RESET_RP_PIN            GPIO_PIN6

/* ------------------------------------------------------------------
 * LED blink code
 * RP state blink: N = rp_state+1 blinks every BLINK_INTERVAL_S seconds.
 * Rapid blink when RP is in grace/reset window.
 * ------------------------------------------------------------------ */
#define BLINK_INTERVAL_S        ( 8U )
#define BLINK_ON_MS             ( 120U )
#define BLINK_OFF_MS            ( 120U )
#define BLINK_GAP_MS            ( 700U )   /* dark pause after sequence */
#define BLINK_RAPID_MS          ( 50U )    /* rapid: RP not responding  */
#define BLINK_RAPID_COUNT       ( 10U )

/* INA219 instance — 3V3_MSP supply line, address 0x41 */
static const ina219_t g_ina_3v3_msp = {
    .addr           = 0x41u,
    .config         = INA219_CFG_3V3_MSP,
    .calib          = INA219_CALIB_3V3_MSP,
    .current_lsb_ua = INA219_LSB_UA_3V3_MSP,
};
/* ------------------------------------------------------------------
 * Rail enable/disable GPIO
 * ------------------------------------------------------------------ */
#define REGA_EN_PORT    GPIO_PORT_P2
#define REGA_EN_PIN     GPIO_PIN4
#define REGB_EN_PORT    GPIO_PORT_P4
#define REGB_EN_PIN     GPIO_PIN4
#define EFUSEA_SHDN_PORT GPIO_PORT_P3
#define EFUSEA_SHDN_PIN  GPIO_PIN2
#define EFUSEB_SHDN_PORT GPIO_PORT_P1
#define EFUSEB_SHDN_PIN  GPIO_PIN5

/* ------------------------------------------------------------------
 * Debugger-accessible rail control globals
 * ------------------------------------------------------------------ */
volatile uint8_t g_rega_en   = 1u;
volatile uint8_t g_regb_en   = 1u;
volatile uint8_t g_efusea_en = 1u;
volatile uint8_t g_efuseb_en = 1u;

/* ------------------------------------------------------------------
 * INA219 results
 * ------------------------------------------------------------------ */
volatile ina219_data_t g_ina_data = { 0, 0, 0, 0 };
volatile uint8_t       g_ina_ok   = 0u;

#pragma PERSISTENT(g_ina_snapshot)
ina219_data_t g_ina_snapshot = { 0, 0, 0, 0 };

#pragma PERSISTENT(g_ina_snapshot_ok)
uint8_t g_ina_snapshot_ok = 0u;

#pragma PERSISTENT(g_ina_diag_init_rc)
int8_t g_ina_diag_init_rc = 0;

#pragma PERSISTENT(g_ina_diag_read_rc)
int8_t g_ina_diag_read_rc = 0;

#pragma PERSISTENT(g_ina_diag_attempts)
uint16_t g_ina_diag_attempts = 0u;

/* ------------------------------------------------------------------
 * RP register snapshot — FRAM persistent, inspect via JTAG after a run.
 * Updated by the CLK task every second from I2C reads.
 * ------------------------------------------------------------------ */
typedef struct {
    uint8_t  hdr_magic[4];         /* bytes from 0x00–0x03 ('H','I','A','1') */
    uint8_t  hdr_version;          /* REG_REGMAP_VERSION (0x04)              */
    uint8_t  hdr_ok;               /* 1 = magic verified on last read        */
    uint8_t  rp_state;             /* REG_RP_STATE (0x10)                    */
    uint32_t rp_uptime_s;          /* REG_RP_UPTIME0–3 (0x11–0x14)           */
    uint16_t rp_hb_counter;        /* REG_RP_HB0–1 (0x15–0x16)              */
    uint8_t  rp_last_error;        /* REG_RP_LAST_ERROR (0x17)               */
    uint8_t  contact_evt_pending;  /* REG_CONTACT_EVT_PENDING (0x1B)         */
    uint8_t  rp_req_flags;         /* REG_RP_REQ_FLAGS (0x30)                */
    uint8_t  rp_req_code;          /* REG_RP_REQ_CODE (0x31)                 */
    uint16_t fault_bitmap;         /* REG_FAULT_BITMAP_L/H (0xF0–0xF1)       */
    uint8_t  fault_seq;            /* REG_FAULT_SEQ (0xF2)                   */
    uint8_t  cnt_i2c_err;          /* REG_CNT_I2C_ERR (0xF5)                 */
    uint32_t last_hdr_ok_second;   /* g_current_second of last good header read */
    uint8_t  i2c_bus_recover_count; /* cumulative Level-2 bus recovery attempts */
} rp_reg_snapshot_t;

#pragma PERSISTENT(g_rp_reg_snapshot)
rp_reg_snapshot_t g_rp_reg_snapshot = { {0,0,0,0}, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0u, 0u };

/* ------------------------------------------------------------------
 * Internal state
 * ------------------------------------------------------------------ */
#pragma NOINIT(g_boot_count)
uint16_t g_boot_count;

#pragma NOINIT(g_current_second)
uint32_t g_current_second;

#pragma PERSISTENT(g_rp_reset_count)
uint16_t g_rp_reset_count = 0u;

/* ------------------------------------------------------------------
 * HIA fault state — PERSISTENT so accumulation survives warm resets.
 * All per-run counters (oc, uv) are reset in main_blinky() each boot;
 * wdt_miss_count and ext_trip carry across reboots by design.
 * ------------------------------------------------------------------ */
typedef struct {
    uint8_t  wdt_miss_count;        /* per-line misses per period; SAFE at >=3      */
    uint8_t  wdt_miss_decay_ctr;    /* seconds of healthy RP; decrement at 60       */
    uint8_t  oc_consec;             /* consecutive OC samples; latch at 3           */
    uint8_t  oc_latched;            /* 1 = OC fault active                          */
    uint8_t  oc_decay_ctr;          /* seconds since last OC; decay at 30           */
    uint8_t  uv_load;               /* 1 = UV_LOAD active (<2900 mV)                */
    uint8_t  ext_trip;              /* sticky: 1 = RST pin asserted at any boot      */
} hia_fault_t;

#pragma PERSISTENT(g_fault)
hia_fault_t g_fault = { 0u, 0u, 0u, 0u, 0u, 0u, 0u };

/* MSP operating mode — PERSISTENT for JTAG inspection of last-run mode.
 * prvModeUpdate() forces STARTUP when g_post_passed==0, so the effective
 * mode on each boot starts at STARTUP regardless of persisted value. */
#pragma PERSISTENT(g_msp_mode)
uint8_t g_msp_mode = MODE_STARTUP;

/* POST gate — non-persistent, re-evaluated each boot from fresh sensor reads */
uint8_t g_post_passed = 0u;

/* RP heartbeat period — read from REG_WDT_PERIOD_S at runtime; default 2 s.
 * WDT miss assessment fires once per this many CLK ticks. */
static uint8_t g_wdt_period_s = 2u;

/* Shared between CLK and Rx: CLK sets flag, Rx consumes it for blink */
static volatile uint8_t g_blink_pending  = 0u;  /* 1 = run blink sequence  */
static volatile uint8_t g_blink_rp_state = 0u;  /* state to blink          */
static volatile uint8_t g_blink_rapid    = 0u;  /* 1 = rapid (RP down)     */

static QueueHandle_t xTickQueue = NULL;

extern volatile uint16_t g_last_sysrstiv;

/* ------------------------------------------------------------------
 * Forward declarations
 * ------------------------------------------------------------------ */
static void prvClockTask  (void *pvParameters);
static void prvRxTask     (void *pvParameters);
static void prvInaTask    (void *pvParameters);

static inline void prvApplyRailEnables(void);
static inline void prvPulseWdt1(void);
static inline void prvPulseWdt2(void);
static inline void prvLedSet(uint8_t on);
static void        prvRunBlinkSequence(void);
static int8_t      prv_rp_read (uint8_t reg, uint8_t *buf,        uint8_t len);
static int8_t      prv_rp_write(uint8_t reg, const uint8_t *data, uint8_t len);
static void        prvFaultUpdate(uint8_t rp_i2c_ok, uint8_t wdt_miss_lines,
                                  int16_t current_ma,
                                  uint16_t bus_mv, uint8_t ina_valid);
static void        prvModeUpdate (uint8_t rp_wdt_grace, uint8_t rp_state);

/* ------------------------------------------------------------------
 * Entry point
 * ------------------------------------------------------------------ */
void main_blinky(void)
{
    uint16_t boot_cause = g_last_sysrstiv;

    if (boot_cause <= SYSRSTIV_DOBOR)
    {
        g_current_second = 0u;
    }
    ++g_boot_count;

    /* Detect external WDT trip (TPS3435 RST pin assertion) — sticky, cleared by ground command */
    if (boot_cause == SYSRSTIV_RSTNMI)
        g_fault.ext_trip = 1u;

    /* RegB and EfuseB hardware is removed — force them off unconditionally */
    g_regb_en   = 0u;
    g_efuseb_en = 0u;

    /* Clear per-run fault state — re-evaluated from fresh sensor readings.
     * wdt_miss_count and ext_trip carry over intentionally. */
    g_fault.uv_load          = 0u;
    g_fault.oc_consec        = 0u;
    g_fault.oc_latched       = 0u;
    g_fault.oc_decay_ctr     = 0u;
    g_fault.wdt_miss_decay_ctr = 0u;

    prvApplyRailEnables();

    xTickQueue = xQueueCreate(8u, sizeof(uint8_t));
    configASSERT(xTickQueue != NULL);

    xTaskCreate(prvClockTask, "CLK", configMINIMAL_STACK_SIZE, NULL, PRIORITY_CLK, NULL);
    xTaskCreate(prvRxTask,    "Rx",  configMINIMAL_STACK_SIZE, NULL, PRIORITY_RX,  NULL);
    xTaskCreate(prvInaTask,   "INA", configMINIMAL_STACK_SIZE, NULL, PRIORITY_INA, NULL);

    vTaskStartScheduler();

    for (;;);
}

/* ------------------------------------------------------------------
 * CLK task — 1 Hz heartbeat, RP regmap I/O
 * ------------------------------------------------------------------ */
static void prvClockTask(void *pvParameters)
{
    TickType_t xNext            = xTaskGetTickCount();
    uint8_t    tick             = 1u;
    uint8_t    rp_wdt_timer     = 0u;    /* counts missed periods; resets on good period */
    uint8_t    rp_wdt_grace     = RP_WDT_GRACE_S;
    uint8_t    wdt_period_ctr   = 0u;    /* CLK ticks within current heartbeat period */
    uint8_t    rp_period_ok     = 0u;    /* cached result from last period assessment */
    uint8_t    blink_counter    = 0u;
    uint8_t    last_cmd_seq     = 0u;    /* track CMD_SEQ written to RP */
    uint8_t    i2c_fail_streak  = 0u;    /* consecutive seconds with failed state read */
    (void)pvParameters;

    /* Convenience macro: write one register to RP with Level-1 retry. */
    #define RP_WRITE(reg, ptr, len) \
        (void)prv_rp_write((reg), (ptr), (len))

    /* prv_rp_read used in place of the old RP_READ macro so the return
     * value is usable in if() conditions. */

    for (;;)
    {
        vTaskDelayUntil(&xNext, pdMS_TO_TICKS(1000u));
        ++g_current_second;

        /* ---- RP watchdog monitor  (evaluated FIRST — determines rp_i2c_ok)
         * Skipping I2C writes when RP is non-responsive prevents multi-second
         * stalls that would starve the Rx task and trigger TPS3435 co-reset.
         * ---------------------------------------------------------------- */
        uint8_t rp_i2c_ok     = 0u;
        uint8_t wdt_miss_lines = 0u;   /* missing WDT lines this period (0, 1, or 2) */

        if (rp_wdt_grace > 0u)
        {
            --rp_wdt_grace;
            if (rp_wdt_grace == 0u)
            {
                /* Grace expired — flush IFG and reset period counter for clean start */
                P3IFG        &= (uint8_t)~(BIT1 | BIT7);
                wdt_period_ctr = 0u;
            }
            /* RP rebooting — hold rp_i2c_ok=0; rp_period_ok already 0 */
        }
        else
        {
            /* Use cached result from last period for all ticks between assessments */
            rp_i2c_ok = rp_period_ok;

            /* Assess lines once per heartbeat period */
            if (++wdt_period_ctr >= g_wdt_period_s)
            {
                wdt_period_ctr = 0u;

                uint8_t rp_ifg   = P3IFG & (BIT1 | BIT7);
                P3IFG           &= (uint8_t)~(BIT1 | BIT7);   /* clear for next period */
                uint8_t line1_ok = (rp_ifg & BIT1) != 0u;
                uint8_t line2_ok = (rp_ifg & BIT7) != 0u;

                if (line1_ok && line2_ok)
                {
                    rp_wdt_timer = 0u;
                    rp_period_ok = 1u;
                    rp_i2c_ok    = 1u;
                }
                else
                {
                    /* Count each absent line as a separate fault event */
                    if (!line1_ok) ++wdt_miss_lines;
                    if (!line2_ok) ++wdt_miss_lines;

                    /* Timeout expressed in periods: RP_WDT_TIMEOUT_S / period_s */
                    uint8_t timeout_periods = (g_wdt_period_s > 0u)
                                              ? (uint8_t)(RP_WDT_TIMEOUT_S / g_wdt_period_s)
                                              : (uint8_t)RP_WDT_TIMEOUT_S;
                    if (timeout_periods == 0u) timeout_periods = 1u;

                    ++rp_wdt_timer;
                    if (rp_wdt_timer >= timeout_periods)
                    {
                        rp_wdt_timer = 0u;
                        ++g_rp_reset_count;
                        GPIO_setOutputLowOnPin(RESET_RP_PORT, RESET_RP_PIN);
                        vTaskDelay(pdMS_TO_TICKS(5u));
                        GPIO_setOutputHighOnPin(RESET_RP_PORT, RESET_RP_PIN);
                        supervisor_i2c_recover();
                        /* MSP initiated this reset — give the RP a clean WDT slate */
                        g_fault.wdt_miss_count     = 0u;
                        g_fault.wdt_miss_decay_ctr = 0u;
                        rp_wdt_grace = RP_WDT_GRACE_S;
                        rp_period_ok = 0u;
                        rp_i2c_ok    = 0u;
                    }
                    else
                    {
                        /* Missed period but timer not expired — RP likely still alive */
                        rp_period_ok = 1u;
                        rp_i2c_ok    = 1u;
                    }
                }
            }
        }

        /* ---- I2C reads from RP (before fault/mode evaluation) ------------- */
        if (rp_i2c_ok)
        {
            uint8_t rd[9];   /* sized for header read 0x00–0x08 */

            /* Header magic + WDT period — check every 4 s */
            if ((g_current_second & 0x03u) == 0u)
            {
                /* Read 9 bytes: 0x00–0x08 covers magic, version, and WDT_PERIOD_S */
                if (prv_rp_read(REG_MAGIC0, rd, 9u) == I2C_OK)
                {
                    memcpy(g_rp_reg_snapshot.hdr_magic, rd, 4u);
                    g_rp_reg_snapshot.hdr_version = rd[4];
                    g_rp_reg_snapshot.hdr_ok =
                        (rd[0] == HDR_MAGIC0_VAL &&
                         rd[1] == HDR_MAGIC1_VAL &&
                         rd[2] == HDR_MAGIC2_VAL &&
                         rd[3] == HDR_MAGIC3_VAL) ? 1u : 0u;
                    if (g_rp_reg_snapshot.hdr_ok)
                        g_rp_reg_snapshot.last_hdr_ok_second = g_current_second;
                    /* rd[8] = REG_WDT_PERIOD_S (offset 0x08 - 0x00) */
                    if (rd[8] >= 1u && rd[8] <= 10u)
                        g_wdt_period_s = rd[8];
                }
            }

            /* RP state/health block (0x10–0x17) — every second.
             * This read is the primary I2C health indicator for the recovery ladder. */
            uint8_t state_read_ok = (prv_rp_read(REG_RP_STATE, rd, 8u) == I2C_OK) ? 1u : 0u;
            if (state_read_ok)
            {
                g_rp_reg_snapshot.rp_state      = rd[0];
                g_rp_reg_snapshot.rp_uptime_s   = (uint32_t)rd[1]
                                                 | ((uint32_t)rd[2] << 8)
                                                 | ((uint32_t)rd[3] << 16)
                                                 | ((uint32_t)rd[4] << 24);
                g_rp_reg_snapshot.rp_hb_counter = (uint16_t)rd[5]
                                                 | ((uint16_t)rd[6] << 8);
                g_rp_reg_snapshot.rp_last_error = rd[7];
                i2c_fail_streak = 0u;
            }
            else
            {
                /* UCB0 may be stuck (isBusBusy=true from a previous STOP timeout).
                 * Reinit UCB0 after 3 consecutive missed reads to unblock the bus. */
                if (++i2c_fail_streak >= 3u)
                {
                    i2c_fail_streak = 0u;
                    supervisor_i2c_recover();
                }
            }

            /* RP request flags (0x30–0x33) — every second */
            if (prv_rp_read(REG_RP_REQ_FLAGS, rd, 4u) == I2C_OK)
            {
                g_rp_reg_snapshot.rp_req_flags = rd[0];
                g_rp_reg_snapshot.rp_req_code  = rd[1];
            }

            /* Fault bitmap (0xF0–0xF5) — every 4 s */
            if ((g_current_second & 0x03u) == 0u)
            {
                if (prv_rp_read(REG_FAULT_BITMAP_L, rd, 6u) == I2C_OK)
                {
                    g_rp_reg_snapshot.fault_bitmap = (uint16_t)rd[0]
                                                   | ((uint16_t)rd[1] << 8);
                    g_rp_reg_snapshot.fault_seq    = rd[2];
                    /* rd[3]=LATCH_FLAGS  rd[4]=reserved  rd[5]=CNT_I2C_ERR */
                    g_rp_reg_snapshot.cnt_i2c_err  = rd[5];
                }
            }

            /* POST gate: set once RP header verified and INA is responding */
            if (!g_post_passed && g_rp_reg_snapshot.hdr_ok && g_ina_ok)
                g_post_passed = 1u;
        }

        /* ---- Fault and mode state machine ---------------------------------- */
        /* Capture INA snapshot atomically for this tick */
        int16_t  tick_ma    = g_ina_data.current_ma;
        uint16_t tick_mv    = g_ina_data.bus_mv;
        uint8_t  tick_ina   = g_ina_ok;

        prvFaultUpdate(rp_i2c_ok, wdt_miss_lines, tick_ma, tick_mv, tick_ina);
        prvModeUpdate(rp_wdt_grace, g_rp_reg_snapshot.rp_state);

        /* ---- Signal Rx task: trigger blink every BLINK_INTERVAL_S seconds */
        if (++blink_counter >= BLINK_INTERVAL_S)
        {
            blink_counter    = 0u;
            g_blink_rp_state = g_rp_reg_snapshot.rp_state;
            g_blink_rapid    = (rp_wdt_grace > 0u) ? 1u : 0u;
            g_blink_pending  = 1u;
        }

        /* ---- Build MSP status bytes ---------------------------------------- */
        uint8_t wdt_ok  = ((P2OUT & BIT2) && (P3OUT & BIT4)) ? 1u : 0u;
        uint8_t rail_ok = (g_rega_en && g_efusea_en) ? 1u : 0u;
        uint8_t batt_ok = tick_ina;

        uint8_t s0 = (uint8_t)(
              (g_msp_mode & STATUS0_MODE_MASK)
            | (wdt_ok  ? STATUS0_WDT_OK  : 0u)
            | (batt_ok ? STATUS0_BATT_OK : 0u)
            | (rail_ok ? STATUS0_RAIL_OK : 0u));

        uint8_t miss_clamped = (g_fault.wdt_miss_count > 63u) ? 63u : g_fault.wdt_miss_count;
        uint8_t s1 = (uint8_t)(
              (g_fault.oc_latched ? STATUS1_OC_MCU  : 0u)
            | (g_fault.uv_load    ? STATUS1_UV_LOAD : 0u)
            | (uint8_t)(miss_clamped << STATUS1_WDT_MISS_SHIFT));

        uint8_t mode_byte = g_msp_mode;

        /* ---- I2C writes to RP (only when RP is known responsive) ----------- */
        if (rp_i2c_ok)
        {
            /* --- MSP status --- */
            RP_WRITE(REG_MSP_STATUS0,     &s0,        1u);
            RP_WRITE(REG_MSP_STATUS1,     &s1,        1u);

            /* --- Contact ack stub (all zeros — no radio on dev board) --- */
            static const uint8_t ack_stub[6] = { 0u, 0u, 0u, 0u, 0u, 0u };
            RP_WRITE(REG_CONTACT_ACK_SEQ, ack_stub,   6u);

            /* --- Mode command --- */
            RP_WRITE(REG_MODE_CMD,        &mode_byte, 1u);

            /* Increment CMD_SEQ so RP can ACK (monotonic, wraps at 255) */
            ++last_cmd_seq;
            RP_WRITE(REG_CMD_SEQ,         &last_cmd_seq, 1u);

            /* --- Telemetry --- */
            RP_WRITE(REG_TLM_MODE,        &mode_byte, 1u);
            RP_WRITE(REG_TLM_MSP_STATUS0, &s0,        1u);
            RP_WRITE(REG_TLM_MSP_STATUS1, &s1,        1u);

            if (tick_ina)
            {
                RP_WRITE(REG_TLM_VBATT_MV, (const uint8_t *)&tick_mv, 2u);
                RP_WRITE(REG_TLM_IBATT_MA, (const uint8_t *)&tick_ma, 2u);
            }

            /* TLM_TBATT_CC — no sensor on dev board, leave as zero */

            /* TLM_UPTIME_S — mirror RP uptime from snapshot (already read above) */
            RP_WRITE(REG_TLM_UPTIME_S, (const uint8_t *)&g_rp_reg_snapshot.rp_uptime_s, 4u);

            /* TLM_CONTACT_AGE_S — stub: no ground contact implemented yet;
             * use seconds-since-boot as proxy age (valid contact never seen) */
            RP_WRITE(REG_TLM_CONTACT_AGE_S, (const uint8_t *)&g_current_second, 4u);

            /* TLM_FAULT_BITMAP_RP — mirror RP fault bitmap from snapshot */
            RP_WRITE(REG_TLM_FAULT_BITMAP_RP, (const uint8_t *)&g_rp_reg_snapshot.fault_bitmap, 2u);
        }

        #undef RP_WRITE

        /* Send tick to Rx task */
        xQueueSend(xTickQueue, &tick, 0u);
    }
}

/* ------------------------------------------------------------------
 * Rx task — LED toggle + WDT pulse + LED blink code
 * ------------------------------------------------------------------ */
static void prvRxTask(void *pvParameters)
{
    uint8_t  tick;
    uint32_t wdt1_counter = 0u;
    uint32_t wdt2_counter = 0u;
    (void)pvParameters;

    for (;;)
    {
        xQueueReceive(xTickQueue, &tick, portMAX_DELAY);

        /* Apply rail enables */
        prvApplyRailEnables();

        /* Pulse WDT1 (P2.2) */
        if (++wdt1_counter >= MSP_WDT_PERIOD_S)
        {
            wdt1_counter = 0u;
            prvPulseWdt1();
        }

        /* Pulse WDT2 (P3.4) */
        if (++wdt2_counter >= MSP_WDT_PERIOD_S)
        {
            wdt2_counter = 0u;
            prvPulseWdt2();
        }

        /* LED blink code — consumes blink_pending flag set by CLK task.
         * Runs blink sequence here (inside Rx) so we can use vTaskDelay
         * without affecting CLK task timing. Maximum blocking ~1.5 s which
         * is within the 2-second WDT pulse window. */
        if (g_blink_pending)
        {
            g_blink_pending = 0u;
            prvRunBlinkSequence();
        }
        else
        {
            /* Normal 1 Hz activity toggle */
            vParTestToggleLED(0);
        }
    }
}

/* ------------------------------------------------------------------
 * INA task — read INA219 @ 0x41 at 2 Hz
 * ------------------------------------------------------------------ */
static void prvInaTask(void *pvParameters)
{
    TickType_t   xNext  = xTaskGetTickCount();
    uint8_t      inited = 0u;
    ina219_data_t data;
    (void)pvParameters;

    if (g_ina_snapshot_ok)
        vTaskDelay(pdMS_TO_TICKS(15000u));

    for (;;)
    {
        vTaskDelayUntil(&xNext, pdMS_TO_TICKS(500u));

        if (!inited)
        {
            int8_t irc = ina219_init(&g_ina_3v3_msp);
            g_ina_diag_init_rc = irc;
            ++g_ina_diag_attempts;
            if (irc != INA219_OK)
                continue;
            inited = 1u;
        }

        {
            int8_t rrc = ina219_read(&g_ina_3v3_msp, &data);
            g_ina_diag_read_rc = rrc;
            if (rrc == INA219_OK)
            {
                g_ina_data        = data;
                g_ina_ok          = 1u;
                g_ina_snapshot    = data;
                g_ina_snapshot_ok = 1u;
            }
            else
            {
                g_ina_ok = 0u;
                inited   = 0u;
            }
        }
    }
}

/* ------------------------------------------------------------------
 * RP I2C helpers — Level-1 retry (up to 3 attempts, 20 ms pause between).
 * The 10 ms post-transaction yield gives CircuitPython time to process.
 * ------------------------------------------------------------------ */
static int8_t prv_rp_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t attempt;
    int8_t  rc = I2C_ERR_TIMEOUT;
    for (attempt = 0u; attempt < 3u; ++attempt)
    {
        rc = i2c_read_reg(RP_I2C_ADDR, reg, buf, len);
        vTaskDelay(pdMS_TO_TICKS(10u));
        if (rc == I2C_OK) return I2C_OK;
        if (attempt < 2u) vTaskDelay(pdMS_TO_TICKS(20u));
    }
    return rc;
}

static int8_t prv_rp_write(uint8_t reg, const uint8_t *data, uint8_t len)
{
    /* Writes retry once only — reduces dead-bus timing slippage.
     * A failed write is acceptable (RP gets data next cycle); reads
     * are where 3-attempt retry matters for data integrity. */
    int8_t rc = i2c_write_reg(RP_I2C_ADDR, reg, data, len);
    vTaskDelay(pdMS_TO_TICKS(10u));
    if (rc == I2C_OK) return I2C_OK;
    vTaskDelay(pdMS_TO_TICKS(20u));
    rc = i2c_write_reg(RP_I2C_ADDR, reg, data, len);
    vTaskDelay(pdMS_TO_TICKS(10u));
    return rc;
}

/* ------------------------------------------------------------------
 * Fault update — called once per CLK tick with fresh sensor data.
 *
 * WDT_RP_MISS  : wdt_miss_lines counts each absent WDT line this tick
 *                (0, 1, or 2); accumulated into wdt_miss_count.  Decays
 *                1 count per 10 s of healthy RP (rp_i2c_ok=1, no misses).
 * OC_MCU       : latches after 3 consecutive samples >100 mA; decays
 *                1 count per 30 s of healthy current; clears at 0.
 * UV_LOAD      : set when bus_mv < 2900, cleared when bus_mv > 3000.
 * EXT_TRIP     : sticky — set in main_blinky() from SYSRSTIV, not here.
 * ------------------------------------------------------------------ */
static void prvFaultUpdate(uint8_t rp_i2c_ok, uint8_t wdt_miss_lines,
                           int16_t current_ma,
                           uint16_t bus_mv, uint8_t ina_valid)
{
    /* --- WDT_RP_MISS ------------------------------------------------ */
    if (wdt_miss_lines > 0u)
    {
        /* Accumulate per-line misses (clamped to 255) */
        if (wdt_miss_lines < (uint8_t)(255u - g_fault.wdt_miss_count))
            g_fault.wdt_miss_count += wdt_miss_lines;
        else
            g_fault.wdt_miss_count = 255u;
        g_fault.wdt_miss_decay_ctr = 0u;   /* restart decay window on miss */
    }
    else if (rp_i2c_ok && (g_fault.wdt_miss_count > 0u))
    {
        if (++g_fault.wdt_miss_decay_ctr >= 10u)
        {
            g_fault.wdt_miss_decay_ctr = 0u;
            --g_fault.wdt_miss_count;
        }
    }

    /* --- OC_MCU ----------------------------------------------------- */
    if (ina_valid)
    {
        if (current_ma > (int16_t)100)
        {
            if (!g_fault.oc_latched)
            {
                if (++g_fault.oc_consec >= 3u)
                    g_fault.oc_latched = 1u;
            }
            g_fault.oc_decay_ctr = 0u;       /* reset decay while OC active */
        }
        else
        {
            if (!g_fault.oc_latched)
            {
                g_fault.oc_consec = 0u;       /* clear counter when healthy */
            }
            else
            {
                /* Latched — decay 1 count per 30 s of healthy current */
                if (++g_fault.oc_decay_ctr >= 30u)
                {
                    g_fault.oc_decay_ctr = 0u;
                    if (g_fault.oc_consec > 0u)
                        --g_fault.oc_consec;
                    if (g_fault.oc_consec == 0u)
                        g_fault.oc_latched = 0u;
                }
            }
        }
    }

    /* --- UV_LOAD ---------------------------------------------------- */
    if (ina_valid)
    {
        if (!g_fault.uv_load && (bus_mv < 2900u))
            g_fault.uv_load = 1u;
        else if (g_fault.uv_load && (bus_mv > 3000u))
            g_fault.uv_load = 0u;
    }
}

/* ------------------------------------------------------------------
 * Mode state machine — called once per CLK tick after prvFaultUpdate.
 *
 * STARTUP  → NOMINAL : POST passed + RP responding nominal + no faults
 * NOMINAL  → SAFE    : wdt_miss_count >= 3  OR  oc_latched
 * SAFE     → NOMINAL : all faults clear + RP responding nominal
 * ANY      → SURVIVAL: uv_load active  (overrides all other transitions;
 *                       power rails are NOT cut — MSP is always-on)
 * SURVIVAL → SAFE    : uv_load cleared
 *
 * While g_post_passed == 0 (not yet verified), mode is forced to STARTUP
 * regardless of persisted g_msp_mode value.
 * ------------------------------------------------------------------ */
static void prvModeUpdate(uint8_t rp_wdt_grace, uint8_t rp_state)
{
    uint8_t rp_ok       = (rp_wdt_grace == 0u) && (rp_state == RP_STATE_NOMINAL);
    uint8_t faults_none = (g_fault.wdt_miss_count < 3u) && !g_fault.oc_latched;

    /* POST gate — mode stays STARTUP until first successful verification */
    if (!g_post_passed)
    {
        g_msp_mode = MODE_STARTUP;
        return;
    }

    /* UV_LOAD: highest priority — push to SURVIVAL from any state */
    if (g_fault.uv_load)
    {
        g_msp_mode = MODE_LOW_POWER;
        return;
    }

    /* Leaving SURVIVAL: safe landing in SAFE (not directly NOMINAL) */
    if (g_msp_mode == MODE_LOW_POWER)
    {
        g_msp_mode = MODE_SAFE;
        return;
    }

    switch (g_msp_mode)
    {
        case MODE_STARTUP:
            if (rp_ok && faults_none)
                g_msp_mode = MODE_NOMINAL;
            break;

        case MODE_NOMINAL:
            if (!faults_none)
                g_msp_mode = MODE_SAFE;
            break;

        case MODE_SAFE:
            if (faults_none && rp_ok)
                g_msp_mode = MODE_NOMINAL;
            break;

        default:
            g_msp_mode = MODE_STARTUP;
            break;
    }
}

/* ------------------------------------------------------------------
 * LED blink sequence
 *
 * Rapid mode (RP down):  BLINK_RAPID_COUNT fast pulses
 * Normal mode:           rp_state+1 pulses at BLINK_ON/OFF_MS cadence,
 *                        then BLINK_GAP_MS dark pause
 *
 * Uses vTaskDelay — Rx task blocks during blink (~1 s max).
 * MSP_WDT_PERIOD_S = 2 s so one missed tick is always safe.
 * ------------------------------------------------------------------ */
static void prvRunBlinkSequence(void)
{
    uint8_t i;

    if (g_blink_rapid)
    {
        for (i = 0u; i < BLINK_RAPID_COUNT; i++)
        {
            prvLedSet(1u);
            vTaskDelay(pdMS_TO_TICKS(BLINK_RAPID_MS));
            prvLedSet(0u);
            vTaskDelay(pdMS_TO_TICKS(BLINK_RAPID_MS));
        }
        return;
    }

    /* N = rp_state + 1, clamped to [1, 8] */
    uint8_t n = (uint8_t)(g_blink_rp_state + 1u);
    if (n > 8u) n = 8u;

    for (i = 0u; i < n; i++)
    {
        prvLedSet(1u);
        vTaskDelay(pdMS_TO_TICKS(BLINK_ON_MS));
        prvLedSet(0u);
        if (i < (uint8_t)(n - 1u))
            vTaskDelay(pdMS_TO_TICKS(BLINK_OFF_MS));
    }
    vTaskDelay(pdMS_TO_TICKS(BLINK_GAP_MS));
}

/* ------------------------------------------------------------------
 * LED direct drive helper (P1.3 = LED0)
 * ------------------------------------------------------------------ */
static inline void prvLedSet(uint8_t on)
{
    if (on)
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
    else
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
}

/* ------------------------------------------------------------------
 * Rail enable helper
 * ------------------------------------------------------------------ */
static inline void prvApplyRailEnables(void)
{
    /* RegA + EfuseA: ALWAYS-ON rail powering the MSP — never cut by mode logic.
     * Debugger override (g_rega_en / g_efusea_en) respected for bench testing. */
    if (g_rega_en)   GPIO_setOutputLowOnPin (REGA_EN_PORT,     REGA_EN_PIN);
    else             GPIO_setOutputHighOnPin(REGA_EN_PORT,     REGA_EN_PIN);

    /* RegB: hardware removed — always off */
    GPIO_setOutputHighOnPin(REGB_EN_PORT, REGB_EN_PIN);

    if (g_efusea_en) GPIO_setOutputLowOnPin (EFUSEA_SHDN_PORT, EFUSEA_SHDN_PIN);
    else             GPIO_setOutputHighOnPin(EFUSEA_SHDN_PORT, EFUSEA_SHDN_PIN);

    /* EfuseB: hardware removed — always off */
    GPIO_setOutputHighOnPin(EFUSEB_SHDN_PORT, EFUSEB_SHDN_PIN);
}

/* ------------------------------------------------------------------
 * WDT pulse helpers
 * ------------------------------------------------------------------ */
static inline void prvPulseWdt1(void)
{
    taskENTER_CRITICAL();
    P2OUT &= (uint8_t)~BIT2;
    __delay_cycles(WDT_LOW_CYCLES);
    P2OUT |= BIT2;
    taskEXIT_CRITICAL();
}

static inline void prvPulseWdt2(void)
{
    taskENTER_CRITICAL();
    P3OUT &= (uint8_t)~BIT4;
    __delay_cycles(WDT_LOW_CYCLES);
    P3OUT |= BIT4;
    taskEXIT_CRITICAL();
}
