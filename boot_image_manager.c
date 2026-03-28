/*
 * boot_image_manager.c
 *
 * RP2350 boot image manager.
 * See boot_image_manager.h for full description.
 */

#include "boot_image_manager.h"
#include "rp_regmap.h"
#include "rp_liveness.h"
#include "supervisor_i2c.h"
#include "mode_state_machine.h"
#include "event_logger.h"
#include "ground_contact.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

/* ------------------------------------------------------------------
 * FRAM NOINIT (survives warm reset, not POR)
 * ------------------------------------------------------------------ */
#ifdef __ICC430__
__no_init volatile uint8_t g_rp_boot_fail_count;
#else
#pragma NOINIT(g_rp_boot_fail_count)
volatile uint8_t g_rp_boot_fail_count;
#endif

/* ------------------------------------------------------------------
 * Internal state
 * ------------------------------------------------------------------ */
typedef enum {
    BOOT_STATE_WAITING = 0u,
    BOOT_STATE_IDLE    = 1u,
} boot_state_t;

/* ------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------ */

static void prvBootImageManagerTask(void *pvParameters)
{
    TickType_t   xNextWake    = xTaskGetTickCount();
    boot_state_t state        = BOOT_STATE_WAITING;
    uint32_t     boot_timer_s = 0u;

    /*
     * Track the last known power-cycle count so we can detect when
     * rp_liveness performs a new power-cycle and restart monitoring.
     */
    uint8_t last_pc_count = g_rp_powercycle_count;

    (void)pvParameters;

    /* Brief startup delay to let rp_liveness power the RP on first */
    vTaskDelay(pdMS_TO_TICKS(2000u));

    for (;;)
    {
        vTaskDelayUntil(&xNextWake, pdMS_TO_TICKS(BOOT_MGR_PERIOD_MS));

        /* Detect a new power-cycle: rp_liveness incremented the counter */
        if (g_rp_powercycle_count != last_pc_count)
        {
            last_pc_count = g_rp_powercycle_count;
            state        = BOOT_STATE_WAITING;
            boot_timer_s = 0u;
        }

        switch (state)
        {
        case BOOT_STATE_WAITING:
        {
            /* Read current RP state register */
            uint8_t rp_state = RP_STATE_INVALID;
            (void)i2c_read_reg_retry(RP_I2C_ADDR, REG_RP_STATE,
                                     &rp_state, 1u);

            if ((rp_state == RP_STATE_NOMINAL) ||
                (rp_state == RP_STATE_SAFE)    ||
                (rp_state == RP_STATE_DEGRADED))
            {
                /* RP booted successfully — confirm the running image.
                 * Use mode_sm_send_boot_cmd() to bump CMD_SEQ so RP
                 * recognises this as a new command even if opcode is unchanged. */
                mode_sm_send_boot_cmd(BOOT_CMD_CONFIRM_CURRENT_IMAGE);

                g_rp_boot_fail_count = 0u;

                event_log_write(EVENT_TYPE_RP_RESET, 4u /* INFO */,
                                (uint32_t)g_contact_age_s,
                                g_rp_powercycle_count, rp_state);

                state = BOOT_STATE_IDLE;
            }
            else
            {
                boot_timer_s++;

                if (boot_timer_s >= BOOT_TIMEOUT_S)
                {
                    /* Boot timed out */
                    g_rp_boot_fail_count++;

                    if (g_rp_boot_fail_count >= BOOT_FAIL_THRESHOLD)
                    {
                        /* Switch to golden image on next power cycle */
                        mode_sm_send_boot_cmd(BOOT_CMD_BOOT_GOLDEN_NEXT);
                        g_rp_boot_fail_count = 0u;

                        event_log_write(EVENT_TYPE_RP_POWERCYCLE, 1u /* P1 */,
                                        (uint32_t)g_contact_age_s,
                                        g_rp_powercycle_count, 0u);
                    }

                    state = BOOT_STATE_IDLE;
                }
            }
        }
        break;

        case BOOT_STATE_IDLE:
            /* Nothing to do until the next power cycle is detected above */
            break;

        default:
            state = BOOT_STATE_WAITING;
            break;
        }
    }
}

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */

void boot_image_manager_task_create(void)
{
    xTaskCreate(prvBootImageManagerTask,
                "BootMgr",
                BOOT_MGR_STACK_SIZE,
                NULL,
                BOOT_MGR_PRIORITY,
                NULL);
}
