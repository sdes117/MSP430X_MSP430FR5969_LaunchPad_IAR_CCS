/*
 * battery_monitor.c
 *
 * Battery monitoring task: voltage/current (1 Hz) + temperature/heater (5 s).
 * See battery_monitor.h for full description.
 */

#include "battery_monitor.h"
#include "power_policy_enforcer.h"
#include "ina219.h"
#include "mcp9808.h"
#include "fault_counters.h"
#include "driverlib.h"
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------
 * Exported battery state
 * ------------------------------------------------------------------ */
volatile uint16_t g_vbatt_mv  = 0u;
volatile int16_t  g_ibatt_ma  = 0;
volatile int16_t  g_tbatt_cc  = -32768;  /* INT16_MIN = "not yet read" */
volatile uint8_t  g_heater_on = 0u;

/* ------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------ */

static void prvHeaterOn(void)
{
    GPIO_setOutputHighOnPin(BATT_HEAT_PORT, BATT_HEAT_PIN);
    g_heater_on = 1u;
}

static void prvHeaterOff(void)
{
    GPIO_setOutputLowOnPin(BATT_HEAT_PORT, BATT_HEAT_PIN);
    g_heater_on = 0u;
}

/* ------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------ */

static void prvBatteryMonitorTask(void *pvParameters)
{
    TickType_t xNextWake        = xTaskGetTickCount();
    uint8_t    temp_divider     = 0u;

    /* Consecutive undervoltage sample counter */
    uint8_t    uv_consec        = 0u;

    /* Overtemp recovery countdown (seconds at or below recovery threshold) */
    uint32_t   ot_recover_s     = 0u;

    /* UV-BATT recovery countdown */
    uint32_t   uv_recover_s     = 0u;

    (void)pvParameters;

    /* Initialise INA219 once; if it fails we'll keep retrying on each poll. */
    (void)ina219_init();

    for (;;)
    {
        vTaskDelayUntil(&xNextWake, pdMS_TO_TICKS(BATTERY_IV_PERIOD_MS));

        /* -------------------------------------------------------
         * 1. Voltage + current (1 Hz via INA219)
         * ------------------------------------------------------- */
        {
            uint16_t vbatt;
            int16_t  ibatt;

            if (ina219_read(&vbatt, &ibatt) == INA219_OK)
            {
                g_vbatt_mv = vbatt;
                g_ibatt_ma = ibatt;

                /* Under-voltage: count consecutive samples below Vcrit */
                if (vbatt < BATT_VCRIT_MV)
                {
                    if (uv_consec < FAULT_UV_BATT_CONSEC_SAMPLES)
                    {
                        uv_consec++;
                    }

                    if (uv_consec >= FAULT_UV_BATT_CONSEC_SAMPLES)
                    {
                        fault_set(MSP_FAULT_PWR_UV_BATT, MSP_FCNT_PWR_UV_BATT);
                        uv_recover_s = 0u;  /* reset recovery countdown */
                    }
                }
                else
                {
                    uv_consec = 0u;

                    /* Recovery: count seconds above Vrec */
                    if ((g_msp_fault_bitmap & MSP_FAULT_PWR_UV_BATT) != 0u)
                    {
                        if (vbatt >= BATT_VREC_MV)
                        {
                            uv_recover_s++;
                            if (uv_recover_s >= FAULT_UV_BATT_RECOVERY_S)
                            {
                                fault_clear(MSP_FAULT_PWR_UV_BATT);
                                uv_recover_s = 0u;
                            }
                        }
                        else
                        {
                            uv_recover_s = 0u;
                        }
                    }
                }
            }
            else
            {
                /* I2C error on INA219 — re-init next cycle */
                (void)ina219_init();
            }
        }

        /* -------------------------------------------------------
         * 2. Temperature + heater (5 s via MCP9808)
         * ------------------------------------------------------- */
        if (++temp_divider >= BATTERY_TEMP_DIVIDER)
        {
            temp_divider = 0u;

            int16_t tbatt;

            if (mcp9808_read_temp(&tbatt) == MCP9808_OK)
            {
                g_tbatt_cc = tbatt;

                /* --- Overtemp fault --- */
                if (tbatt > BATT_TEMP_OVERTEMP_CC)
                {
                    fault_set(MSP_FAULT_THERM_OVERTEMP, MSP_FCNT_THERM_OVERTEMP);
                    ot_recover_s = 0u;
                }
                else if ((g_msp_fault_bitmap & MSP_FAULT_THERM_OVERTEMP) != 0u)
                {
                    /* Count time below recovery threshold */
                    if (tbatt <= BATT_TEMP_OVERTEMP_REC_CC)
                    {
                        ot_recover_s += BATTERY_TEMP_DIVIDER;
                        if (ot_recover_s >= FAULT_OVERTEMP_RECOVERY_S)
                        {
                            fault_clear(MSP_FAULT_THERM_OVERTEMP);
                            ot_recover_s = 0u;
                        }
                    }
                    else
                    {
                        ot_recover_s = 0u;
                    }
                }

                /* --- Low-temp fault (drives heater) --- */
                if (tbatt < BATT_TEMP_HEAT_ON_CC)
                {
                    fault_set(MSP_FAULT_THERM_BATT_LOW, MSP_FCNT_THERM_BATT_LOW);
                }
                else if (tbatt >= BATT_TEMP_HEAT_OFF_CC)
                {
                    fault_clear(MSP_FAULT_THERM_BATT_LOW);
                }
                /* Between HEAT_ON and HEAT_OFF: hysteresis — no change to fault */

                /* --- Heater control (hysteresis, subject to policy permission) --- */
                if (tbatt < BATT_TEMP_HEAT_ON_CC)
                {
                    if (g_heater_permitted != 0u)
                    {
                        prvHeaterOn();
                    }
                    else if (g_heater_on != 0u)
                    {
                        /* Permission revoked while heater was on — force off */
                        prvHeaterOff();
                    }
                }
                else if (tbatt >= BATT_TEMP_HEAT_OFF_CC)
                {
                    prvHeaterOff();
                }
                /* In the hysteresis band: leave heater state unchanged,
                 * but still enforce permission revocation. */
                else if ((g_heater_permitted == 0u) && (g_heater_on != 0u))
                {
                    prvHeaterOff();
                }
            }
        }
    }
}

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */

void battery_monitor_task_create(void)
{
    xTaskCreate(prvBatteryMonitorTask,
                "BattMon",
                BATTERY_STACK_SIZE,
                NULL,
                BATTERY_PRIORITY,
                NULL);
}
