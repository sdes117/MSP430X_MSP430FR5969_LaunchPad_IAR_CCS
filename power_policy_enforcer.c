/*
 * power_policy_enforcer.c
 *
 * MSP-local power rail GPIO policy task.
 * See power_policy_enforcer.h for full description.
 *
 * eFuse FLT monitoring and cycling is handled by efuse_cycle.c.
 * Regulator PG monitoring and cycling is handled by reg_cycle.c.
 */

#include "power_policy_enforcer.h"
#include "mode_state_machine.h"
#include "driverlib.h"
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------
 * Exported state
 * ------------------------------------------------------------------ */
volatile uint8_t g_heater_permitted = 1u;  /* permitted by default */

/* ------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------ */

static void prvRailsAllOn(void)
{
    GPIO_setOutputLowOnPin(REGA_EN_PORT,     REGA_EN_PIN);
    GPIO_setOutputLowOnPin(REGB_EN_PORT,     REGB_EN_PIN);
    GPIO_setOutputLowOnPin(EFUSEA_SHDN_PORT, EFUSEA_SHDN_PIN);
    GPIO_setOutputLowOnPin(EFUSEB_SHDN_PORT, EFUSEB_SHDN_PIN);
}

static void prvRailsRegBOff(void)
{
    /* RegA + eFuseA remain enabled */
    GPIO_setOutputLowOnPin(REGA_EN_PORT,     REGA_EN_PIN);
    GPIO_setOutputLowOnPin(EFUSEA_SHDN_PORT, EFUSEA_SHDN_PIN);
    /* RegB + eFuseB disabled */
    GPIO_setOutputHighOnPin(REGB_EN_PORT,    REGB_EN_PIN);
    GPIO_setOutputHighOnPin(EFUSEB_SHDN_PORT, EFUSEB_SHDN_PIN);
}

static void prvRailsAllOff(void)
{
    GPIO_setOutputHighOnPin(REGA_EN_PORT,     REGA_EN_PIN);
    GPIO_setOutputHighOnPin(REGB_EN_PORT,     REGB_EN_PIN);
    GPIO_setOutputHighOnPin(EFUSEA_SHDN_PORT, EFUSEA_SHDN_PIN);
    GPIO_setOutputHighOnPin(EFUSEB_SHDN_PORT, EFUSEB_SHDN_PIN);
}

/* ------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------ */

static void prvPowerPolicyTask(void *pvParameters)
{
    TickType_t xNextWake = xTaskGetTickCount();

    (void)pvParameters;

    for (;;)
    {
        vTaskDelayUntil(&xNextWake, pdMS_TO_TICKS(POWER_POLICY_PERIOD_MS));

        /* Apply rail GPIO policy based on current MSP mode */
        switch (g_msp_mode)
        {
        case MSP_MODE_STARTUP:
        case MSP_MODE_NOMINAL:
            prvRailsAllOn();
            g_heater_permitted = 1u;
            break;

        case MSP_MODE_SAFE:
            prvRailsRegBOff();
            g_heater_permitted = 1u;
            break;

        case MSP_MODE_SURVIVAL:
            prvRailsAllOff();
            g_heater_permitted = 0u;
            break;

        default:
            prvRailsAllOff();
            g_heater_permitted = 0u;
            break;
        }
    }
}

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */

void power_policy_enforcer_task_create(void)
{
    xTaskCreate(prvPowerPolicyTask,
                "PwrPol",
                POWER_POLICY_STACK_SIZE,
                NULL,
                POWER_POLICY_PRIORITY,
                NULL);
}
