/*
 * mode_state_machine.c
 *
 * Supervisor mode state machine and power policy enforcer.
 * See mode_state_machine.h for full description.
 *
 * Transitions (Mode Authority Matrix):
 *
 *   STARTUP  -> NOMINAL   : After MODE_SM_STARTUP_POLLS clean polls,
 *                           no P0 faults, RP live.
 *   NOMINAL  -> SAFE      : Any P0 fault active  OR  RP entering GPIO stall.
 *   SAFE     -> SURVIVAL  : Battery critically low (F_PWR_UV_BATT) OR
 *                           overtemp (F_THERM_OVERTEMP)             OR
 *                           RP power-cycle limit reached.
 *   SURVIVAL             : RP powered off; only thermal/battery monitored.
 *                           Exit only on full MSP reset.
 *   Any mode -> SAFE      : RP recovers from SURVIVAL state (re-powered).
 */

#include "mode_state_machine.h"
#include "battery_monitor.h"
#include "fault_counters.h"
#include "rp_liveness.h"
#include "rp_regmap.h"
#include "supervisor_i2c.h"
#include "event_logger.h"
#include "ground_contact.h"
#include "FreeRTOS.h"
#include "task.h"
#include "driverlib.h"

/* ------------------------------------------------------------------
 * Exported state
 * ------------------------------------------------------------------ */
volatile msp_mode_t g_msp_mode  = MSP_MODE_STARTUP;
volatile uint8_t    g_rail_mask = RAIL_MASK_SURVIVAL;  /* safe default at boot */

/* ------------------------------------------------------------------
 * Private helpers
 * ------------------------------------------------------------------ */

/* Monotonic sequence counter for the mode/power command block. */
static uint8_t s_mode_cmd_seq = 0u;

/* Map MSP supervisor mode to the MODE_CMD value RP expects. */
static uint8_t prvModeToCmd(msp_mode_t mode)
{
    switch (mode)
    {
    case MSP_MODE_NOMINAL:  return MODE_CMD_ENTER_NOMINAL;
    case MSP_MODE_SAFE:     return MODE_CMD_ENTER_SAFE;
    case MSP_MODE_SURVIVAL: return MODE_CMD_PREPARE_FOR_SURVIVAL;
    default:                return MODE_CMD_ENTER_STARTUP_MINIMUM;
    }
}

/*
 * Write the full 7-byte mode/power command block (REG_MODE_CMD..REG_CMD_SEQ)
 * to RP in a single I2C transaction so RP sees an atomic snapshot.
 *
 *   [0x28] MODE_CMD   — desired RP operating mode
 *   [0x29] POWER_CMD  — APPLY_RAIL_MASK (enforce the supplied mask)
 *   [0x2A] RESET_CMD  — NO_OP
 *   [0x2B] BOOT_CMD   — NO_OP
 *   [0x2C] RAIL_MASK_LO
 *   [0x2D] RAIL_MASK_HI (always 0 — upper rails unused)
 *   [0x2E] CMD_SEQ    — monotonically incremented; RP uses this to detect
 *                       new commands vs. repeated identical writes.
 */
static void prvPublishModeCmd(msp_mode_t mode, uint8_t mask)
{
    uint8_t buf[7];

    s_mode_cmd_seq++;

    buf[0] = prvModeToCmd(mode);            /* MODE_CMD  */
    buf[1] = POWER_CMD_APPLY_RAIL_MASK;     /* POWER_CMD */
    buf[2] = RESET_CMD_NO_OP;               /* RESET_CMD */
    buf[3] = BOOT_CMD_NO_OP;                /* BOOT_CMD  */
    buf[4] = mask;                          /* RAIL_MASK_LO */
    buf[5] = 0x00u;                         /* RAIL_MASK_HI  */
    buf[6] = s_mode_cmd_seq;                /* CMD_SEQ   */

    (void)i2c_write_reg(RP_I2C_ADDR, REG_MODE_CMD, buf, 7u);
}

/* Return 1 if any P0 fault is currently active. */
static uint8_t prvP0FaultActive(void)
{
    static const uint16_t P0_MASK =
          MSP_FAULT_PWR_UV_BATT
        | MSP_FAULT_WDT_MSP_MISS
        | MSP_FAULT_WDT_EXT_TRIP
        | MSP_FAULT_WDT_RP_MISS
        | MSP_FAULT_THERM_OVERTEMP
        | MSP_FAULT_RP_TX_SHUTDOWN;  /* regulatory P0: RP permanent TX shutdown */

    return ((g_msp_fault_bitmap & P0_MASK) != 0u) ? 1u : 0u;
}

/* ------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------ */

static void prvModeStateMachineTask(void *pvParameters)
{
    TickType_t xNextWake    = xTaskGetTickCount();
    uint8_t    startup_polls = 0u;
    msp_mode_t prev_mode    = MSP_MODE_STARTUP;

    (void)pvParameters;

    for (;;)
    {
        vTaskDelayUntil(&xNextWake, pdMS_TO_TICKS(MODE_SM_PERIOD_MS));

        prev_mode = g_msp_mode;

        switch (g_msp_mode)
        {
        /* ---- STARTUP -------------------------------------------- */
        case MSP_MODE_STARTUP:
            /*
             * Stay in STARTUP until we have a few clean polls:
             * - No P0 faults
             * - RP is reachable (liveness task in OK or I2C_STALL state)
             */
            if ((prvP0FaultActive() == 0u) &&
                (g_rp_live_state <= RP_LIVE_I2C_STALL))
            {
                startup_polls++;
                if (startup_polls >= MODE_SM_STARTUP_POLLS)
                {
                    g_msp_mode  = MSP_MODE_NOMINAL;
                    g_rail_mask = RAIL_MASK_NOMINAL;
                    prvPublishModeCmd(g_msp_mode, g_rail_mask);
                }
            }
            else
            {
                startup_polls = 0u;
            }
            break;

        /* ---- NOMINAL -------------------------------------------- */
        case MSP_MODE_NOMINAL:
            if ((g_msp_fault_bitmap & MSP_FAULT_PWR_UV_BATT) != 0u ||
                (g_msp_fault_bitmap & MSP_FAULT_THERM_OVERTEMP) != 0u ||
                (g_rp_powercycle_count >= RP_POWERCYCLE_MAX_ATTEMPTS))
            {
                /* Escalate straight to SURVIVAL */
                g_msp_mode  = MSP_MODE_SURVIVAL;
                g_rail_mask = RAIL_MASK_SURVIVAL;
                prvPublishModeCmd(g_msp_mode, g_rail_mask);
            }
            else if (prvP0FaultActive() != 0u ||
                     g_rp_live_state >= RP_LIVE_GPIO_STALL)
            {
                g_msp_mode  = MSP_MODE_SAFE;
                g_rail_mask = RAIL_MASK_SAFE;
                prvPublishModeCmd(g_msp_mode, g_rail_mask);
            }
            break;

        /* ---- SAFE ----------------------------------------------- */
        case MSP_MODE_SAFE:
            if ((g_msp_fault_bitmap & MSP_FAULT_PWR_UV_BATT) != 0u ||
                (g_msp_fault_bitmap & MSP_FAULT_THERM_OVERTEMP) != 0u ||
                (g_rp_powercycle_count >= RP_POWERCYCLE_MAX_ATTEMPTS))
            {
                g_msp_mode  = MSP_MODE_SURVIVAL;
                g_rail_mask = RAIL_MASK_SURVIVAL;
                prvPublishModeCmd(g_msp_mode, g_rail_mask);
            }
            else if (prvP0FaultActive() == 0u &&
                     g_rp_live_state == RP_LIVE_OK)
            {
                /* All faults cleared and RP healthy — recover to NOMINAL */
                g_msp_mode  = MSP_MODE_NOMINAL;
                g_rail_mask = RAIL_MASK_NOMINAL;
                prvPublishModeCmd(g_msp_mode, g_rail_mask);
            }
            break;

        /* ---- SURVIVAL ------------------------------------------- */
        case MSP_MODE_SURVIVAL:
            /*
             * Remain here permanently. RP is powered off (managed by
             * rp_liveness). Only heater and battery remain active.
             * Rail mask stays SURVIVAL; only a full MSP reset can exit.
             */
            g_rail_mask = RAIL_MASK_SURVIVAL;
            break;

        default:
            g_msp_mode = MSP_MODE_STARTUP;
            break;
        }

        /* -------------------------------------------------------
         * Check CMD_STATUS (0x2F) — log if RP rejected our last
         * mode/power command.  Skip in SURVIVAL (RP is off).
         * ------------------------------------------------------- */
        if (g_msp_mode != MSP_MODE_SURVIVAL)
        {
            uint8_t cmd_status = 0u;
            if (i2c_read_reg(RP_I2C_ADDR, REG_CMD_STATUS,
                             &cmd_status, 1u) == I2C_OK)
            {
                if (cmd_status & CMD_STATUS_REJECTED)
                {
                    event_log_write(EVENT_TYPE_FAULT_SET, 2u /* P2 */,
                                    (uint32_t)g_contact_age_s,
                                    cmd_status, s_mode_cmd_seq);
                }
            }
        }

        /* Log mode transitions for msp_survival_event_logger requirement.
         * Works in SURVIVAL (no RP needed — writes to FRAM only). */
        if (g_msp_mode != prev_mode)
        {
            uint8_t sev = (g_msp_mode == MSP_MODE_SURVIVAL) ? 0u :  /* P0 */
                          (g_msp_mode == MSP_MODE_SAFE)     ? 1u :  /* P1 */
                          4u;                                         /* INFO */

            event_log_write(EVENT_TYPE_MODE_CHANGE, sev,
                            (uint32_t)g_contact_age_s,
                            (uint8_t)prev_mode,
                            (uint8_t)g_msp_mode);

            /* Dedicated survival entry event for easy log filtering */
            if (g_msp_mode == MSP_MODE_SURVIVAL)
            {
                event_log_write(EVENT_TYPE_RP_SURVIVAL, 0u,
                                (uint32_t)g_contact_age_s, 0u, 0u);
            }
        }
    }
}

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */

void mode_state_machine_task_create(void)
{
    xTaskCreate(prvModeStateMachineTask,
                "ModeSM",
                MODE_SM_STACK_SIZE,
                NULL,
                MODE_SM_PRIORITY,
                NULL);
}

/* ------------------------------------------------------------------
 * Public: write a BOOT_CMD through the full mode/power block
 * ------------------------------------------------------------------ */

void mode_sm_send_boot_cmd(uint8_t boot_cmd)
{
    uint8_t buf[7];

    s_mode_cmd_seq++;

    buf[0] = prvModeToCmd(g_msp_mode);      /* MODE_CMD  — current mode */
    buf[1] = POWER_CMD_APPLY_RAIL_MASK;     /* POWER_CMD */
    buf[2] = RESET_CMD_NO_OP;               /* RESET_CMD */
    buf[3] = boot_cmd;                      /* BOOT_CMD  — caller-specified */
    buf[4] = g_rail_mask;                   /* RAIL_MASK_LO */
    buf[5] = 0x00u;                         /* RAIL_MASK_HI */
    buf[6] = s_mode_cmd_seq;                /* CMD_SEQ   */

    (void)i2c_write_reg(RP_I2C_ADDR, REG_MODE_CMD, buf, 7u);
}
