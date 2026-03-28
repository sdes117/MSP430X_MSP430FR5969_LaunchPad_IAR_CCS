/*
 * efuse_cycle.c
 *
 * eFuse fault monitor and recovery cycling task.
 * See efuse_cycle.h for full description.
 */

#include "efuse_cycle.h"
#include "mode_state_machine.h"
#include "fault_counters.h"
#include "driverlib.h"
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------
 * Internal: cycle one eFuse's SHDN pin to reset the latch
 * ------------------------------------------------------------------ */

static void prvCycleEfuse(uint8_t shdn_port, uint8_t shdn_pin)
{
    /* Assert SHDN to reset the eFuse latch */
    GPIO_setOutputHighOnPin(shdn_port, shdn_pin);
    vTaskDelay(pdMS_TO_TICKS(EFUSE_CYCLE_OFF_MS));
    /* Release SHDN to re-enable the eFuse */
    GPIO_setOutputLowOnPin(shdn_port, shdn_pin);
    vTaskDelay(pdMS_TO_TICKS(EFUSE_CYCLE_SETTLE_MS));
}

/* ------------------------------------------------------------------
 * Internal: per-eFuse state
 * ------------------------------------------------------------------ */

typedef struct {
    uint8_t flt_count;   /* consecutive FLT-asserted polls  */
    uint8_t flt_stable;  /* consecutive FLT-deasserted polls */
    uint8_t attempts;    /* cycle attempts since last trip   */
} efuse_state_t;

/* ------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------ */

static void prvEfuseCycleTask(void *pvParameters)
{
    TickType_t    xNextWake = xTaskGetTickCount();
    efuse_state_t efA = {0u, 0u, 0u};
    efuse_state_t efB = {0u, 0u, 0u};

    (void)pvParameters;

    for (;;)
    {
        vTaskDelayUntil(&xNextWake, pdMS_TO_TICKS(EFUSE_CYCLE_PERIOD_MS));

        /* Determine which eFuses to monitor based on current mode */
        uint8_t monA = ((g_msp_mode == MSP_MODE_NOMINAL) ||
                        (g_msp_mode == MSP_MODE_SAFE));
        uint8_t monB =  (g_msp_mode == MSP_MODE_NOMINAL);

        /* -------------------------------------------------------
         * eFuseA monitoring
         * ------------------------------------------------------- */
        if (monA)
        {
            /* ~FLT: LOW = fault asserted */
            uint8_t flt = GPIO_getInputPinValue(EFUSEA_FLT_PORT, EFUSEA_FLT_PIN);

            if (flt != 0u)
            {
                /* FLT de-asserted — eFuse healthy */
                efA.flt_count = 0u;
                efA.attempts  = 0u;
                efA.flt_stable++;

                if ((efA.flt_stable >= EFUSE_FLT_STABLE_POLLS) &&
                    ((g_msp_fault_bitmap & MSP_FAULT_PWR_OC_MCU) != 0u))
                {
                    /* Only clear if eFuseB is also healthy */
                    uint8_t fltB = GPIO_getInputPinValue(EFUSEB_FLT_PORT,
                                                         EFUSEB_FLT_PIN);
                    if (!monB || (fltB != 0u))
                    {
                        fault_clear(MSP_FAULT_PWR_OC_MCU);
                    }
                }
            }
            else
            {
                /* FLT asserted — overcurrent or thermal trip */
                efA.flt_stable = 0u;
                efA.flt_count++;

                if (efA.flt_count >= EFUSE_FLT_TRIP_THRESHOLD)
                {
                    fault_set(MSP_FAULT_PWR_OC_MCU, MSP_FCNT_PWR_OC_MCU);

                    if (efA.attempts < EFUSE_CYCLE_MAX_ATTEMPTS)
                    {
                        efA.attempts++;
                        efA.flt_count = 0u;
                        prvCycleEfuse(EFUSEA_SHDN_PORT, EFUSEA_SHDN_PIN);
                    }
                    /* Beyond max attempts: fault remains set; no further
                     * cycling until FLT naturally de-asserts. */
                }
            }
        }
        else
        {
            efA.flt_count  = 0u;
            efA.flt_stable = 0u;
            efA.attempts   = 0u;
        }

        /* -------------------------------------------------------
         * eFuseB monitoring
         * ------------------------------------------------------- */
        if (monB)
        {
            uint8_t flt = GPIO_getInputPinValue(EFUSEB_FLT_PORT, EFUSEB_FLT_PIN);

            if (flt != 0u)
            {
                efB.flt_count = 0u;
                efB.attempts  = 0u;
                efB.flt_stable++;

                if ((efB.flt_stable >= EFUSE_FLT_STABLE_POLLS) &&
                    ((g_msp_fault_bitmap & MSP_FAULT_PWR_OC_MCU) != 0u))
                {
                    uint8_t fltA = GPIO_getInputPinValue(EFUSEA_FLT_PORT,
                                                         EFUSEA_FLT_PIN);
                    if (fltA != 0u)
                    {
                        fault_clear(MSP_FAULT_PWR_OC_MCU);
                    }
                }
            }
            else
            {
                efB.flt_stable = 0u;
                efB.flt_count++;

                if (efB.flt_count >= EFUSE_FLT_TRIP_THRESHOLD)
                {
                    fault_set(MSP_FAULT_PWR_OC_MCU, MSP_FCNT_PWR_OC_MCU);

                    if (efB.attempts < EFUSE_CYCLE_MAX_ATTEMPTS)
                    {
                        efB.attempts++;
                        efB.flt_count = 0u;
                        prvCycleEfuse(EFUSEB_SHDN_PORT, EFUSEB_SHDN_PIN);
                    }
                }
            }
        }
        else
        {
            efB.flt_count  = 0u;
            efB.flt_stable = 0u;
            efB.attempts   = 0u;
        }
    }
}

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */

void efuse_cycle_task_create(void)
{
    xTaskCreate(prvEfuseCycleTask,
                "EfCyc",
                EFUSE_CYCLE_STACK_SIZE,
                NULL,
                EFUSE_CYCLE_PRIORITY,
                NULL);
}
