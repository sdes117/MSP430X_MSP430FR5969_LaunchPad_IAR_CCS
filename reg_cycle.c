/*
 * reg_cycle.c
 *
 * Regulator power-good monitor and recovery cycling task.
 * See reg_cycle.h for full description.
 */

#include "reg_cycle.h"
#include "power_policy_enforcer.h"   /* REGA_EN_*, REGB_EN_* pin defines */
#include "mode_state_machine.h"
#include "fault_counters.h"
#include "driverlib.h"
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------
 * Internal: cycle one regulator's enable pin
 * Caller must ensure the rail is supposed to be ON in the current mode.
 * ------------------------------------------------------------------ */

static void prvCycleReg(uint8_t en_port, uint8_t en_pin)
{
    /* Disable */
    GPIO_setOutputHighOnPin(en_port, en_pin);
    vTaskDelay(pdMS_TO_TICKS(REG_CYCLE_OFF_MS));
    /* Re-enable */
    GPIO_setOutputLowOnPin(en_port, en_pin);
    vTaskDelay(pdMS_TO_TICKS(REG_CYCLE_SETTLE_MS));
}

/* ------------------------------------------------------------------
 * Internal: per-rail state
 * ------------------------------------------------------------------ */

typedef struct {
    uint8_t pg_miss;     /* consecutive PG-low polls         */
    uint8_t pg_stable;   /* consecutive PG-high polls         */
    uint8_t attempts;    /* cycle attempts since last PG-loss */
} rail_state_t;

/* ------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------ */

static void prvRegCycleTask(void *pvParameters)
{
    TickType_t   xNextWake = xTaskGetTickCount();
    rail_state_t regA = {0u, 0u, 0u};
    rail_state_t regB = {0u, 0u, 0u};

    (void)pvParameters;

    for (;;)
    {
        vTaskDelayUntil(&xNextWake, pdMS_TO_TICKS(REG_CYCLE_PERIOD_MS));

        /* Determine which rails to monitor based on current mode */
        uint8_t monA = ((g_msp_mode == MSP_MODE_NOMINAL) ||
                        (g_msp_mode == MSP_MODE_SAFE));
        uint8_t monB =  (g_msp_mode == MSP_MODE_NOMINAL);

        /* -------------------------------------------------------
         * RegA monitoring
         * ------------------------------------------------------- */
        if (monA)
        {
            uint8_t pg = GPIO_getInputPinValue(REGA_PG_PORT, REGA_PG_PIN);

            if (pg != 0u)
            {
                /* PG OK — count stable polls toward fault clear */
                regA.pg_miss    = 0u;
                regA.attempts   = 0u;
                regA.pg_stable++;
                if ((regA.pg_stable >= REG_PG_STABLE_POLLS) &&
                    ((g_msp_fault_bitmap & MSP_FAULT_PWR_UV_LOAD) != 0u))
                {
                    fault_clear(MSP_FAULT_PWR_UV_LOAD);
                }
            }
            else
            {
                regA.pg_stable = 0u;
                regA.pg_miss++;

                if (regA.pg_miss >= REG_PG_MISS_THRESHOLD)
                {
                    fault_set(MSP_FAULT_PWR_UV_LOAD, MSP_FCNT_PWR_UV_LOAD);

                    if (regA.attempts < REG_CYCLE_MAX_ATTEMPTS)
                    {
                        regA.attempts++;
                        regA.pg_miss = 0u;
                        prvCycleReg(REGA_EN_PORT, REGA_EN_PIN);
                    }
                    /* If max attempts reached, leave the fault set and
                     * stop cycling until PG recovers naturally. */
                }
            }
        }
        else
        {
            /* Rail not active — reset counters */
            regA.pg_miss   = 0u;
            regA.pg_stable = 0u;
            regA.attempts  = 0u;
        }

        /* -------------------------------------------------------
         * RegB monitoring
         * ------------------------------------------------------- */
        if (monB)
        {
            uint8_t pg = GPIO_getInputPinValue(REGB_PG_PORT, REGB_PG_PIN);

            if (pg != 0u)
            {
                regB.pg_miss   = 0u;
                regB.attempts  = 0u;
                regB.pg_stable++;
                if ((regB.pg_stable >= REG_PG_STABLE_POLLS) &&
                    ((g_msp_fault_bitmap & MSP_FAULT_PWR_UV_LOAD) != 0u))
                {
                    /* Only clear if RegA is also OK */
                    uint8_t pgA = GPIO_getInputPinValue(REGA_PG_PORT, REGA_PG_PIN);
                    if (pgA != 0u)
                    {
                        fault_clear(MSP_FAULT_PWR_UV_LOAD);
                    }
                }
            }
            else
            {
                regB.pg_stable = 0u;
                regB.pg_miss++;

                if (regB.pg_miss >= REG_PG_MISS_THRESHOLD)
                {
                    fault_set(MSP_FAULT_PWR_UV_LOAD, MSP_FCNT_PWR_UV_LOAD);

                    if (regB.attempts < REG_CYCLE_MAX_ATTEMPTS)
                    {
                        regB.attempts++;
                        regB.pg_miss = 0u;
                        prvCycleReg(REGB_EN_PORT, REGB_EN_PIN);
                    }
                }
            }
        }
        else
        {
            regB.pg_miss   = 0u;
            regB.pg_stable = 0u;
            regB.attempts  = 0u;
        }
    }
}

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */

void reg_cycle_task_create(void)
{
    xTaskCreate(prvRegCycleTask,
                "RegCyc",
                REG_CYCLE_STACK_SIZE,
                NULL,
                REG_CYCLE_PRIORITY,
                NULL);
}
