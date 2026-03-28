/*
 * eps_3v3_monitor.c
 *
 * EPS 3.3 V rail current-limit monitor for the RP2350 supply.
 * See eps_3v3_monitor.h for full description.
 */

#include "eps_3v3_monitor.h"
#include "supervisor_i2c.h"
#include "fault_counters.h"
#include "rp_liveness.h"
#include "driverlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

/* ------------------------------------------------------------------
 * INA219 register addresses (same hardware as ina219.h, different device)
 * ------------------------------------------------------------------ */
#define REG_CONFIG   (0x00u)
#define REG_CURRENT  (0x04u)
#define REG_CALIB    (0x05u)

/* Configuration: bus 32 V range, gain /8 (±320 mV), 12-bit, continuous */
#define INA219_CFG   (0x3FFFu)

/* ------------------------------------------------------------------
 * Internal: write a 16-bit register to the 3V3 INA219 (big-endian)
 * ------------------------------------------------------------------ */
static int8_t prv_write16(uint8_t reg, uint16_t val)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(val >> 8u);
    buf[1] = (uint8_t)(val & 0xFFu);
    return i2c_write_reg(EPS_3V3_INA219_ADDR, reg, buf, 2u);
}

/* ------------------------------------------------------------------
 * Internal: read current in milliamps from the 3V3 INA219.
 * Returns I2C_OK on success; current_ma_out is signed (positive = load).
 * ------------------------------------------------------------------ */
static int8_t prv_read_current_ma(int16_t *current_ma_out)
{
    uint8_t buf[2] = {0u, 0u};
    int8_t  rc;

    rc = i2c_read_reg(EPS_3V3_INA219_ADDR, REG_CURRENT, buf, 2u);
    if (rc != I2C_OK)
    {
        return rc;
    }

    {
        int16_t raw   = (int16_t)(((uint16_t)buf[0] << 8u) | (uint16_t)buf[1]);
        int32_t ua    = (int32_t)raw * (int32_t)EPS_3V3_INA219_CURRENT_LSB_UA;
        *current_ma_out = (int16_t)(ua / 1000);
    }

    return I2C_OK;
}

/* ------------------------------------------------------------------
 * Internal: EN_3V3 and RESET_RP helpers
 * ------------------------------------------------------------------ */

static void prv_rp_power_off(void)
{
    /* Assert reset before cutting power */
    GPIO_setOutputLowOnPin(EPS_3V3_RESET_RP_PORT, EPS_3V3_RESET_RP_PIN);
    GPIO_setOutputLowOnPin(EPS_3V3_EN_PORT, EPS_3V3_EN_PIN);
}

static void prv_rp_power_on(void)
{
    GPIO_setOutputHighOnPin(EPS_3V3_EN_PORT, EPS_3V3_EN_PIN);
    vTaskDelay(pdMS_TO_TICKS(EPS_3V3_SETTLE_MS));
    GPIO_setOutputHighOnPin(EPS_3V3_RESET_RP_PORT, EPS_3V3_RESET_RP_PIN);
}

/* ------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------ */

static void prvEps3V3MonitorTask(void *pvParameters)
{
    TickType_t xNextWake = xTaskGetTickCount();
    uint8_t    oc_count    = 0u;
    uint8_t    stable_count = 0u;
    uint8_t    attempts    = 0u;

    (void)pvParameters;

    /* Initialise the 3V3 REG INA219 */
    (void)prv_write16(REG_CONFIG, INA219_CFG);
    (void)prv_write16(REG_CALIB,  EPS_3V3_INA219_CALIB);

    for (;;)
    {
        vTaskDelayUntil(&xNextWake, pdMS_TO_TICKS(EPS_3V3_POLL_MS));

        int16_t current_ma = 0;
        int8_t  rc = prv_read_current_ma(&current_ma);

        if (rc != I2C_OK)
        {
            /* I2C failure — skip this poll; let i2c_census / rp_liveness
             * handle the bus fault. */
            oc_count = 0u;
            continue;
        }

        if (current_ma > (int16_t)EPS_3V3_OC_THRESHOLD_MA)
        {
            stable_count = 0u;
            oc_count++;

            if (oc_count >= EPS_3V3_OC_TRIP_THRESHOLD)
            {
                fault_set(MSP_FAULT_PWR_UV_LOAD, MSP_FCNT_PWR_UV_LOAD);

                if (attempts < EPS_3V3_MAX_ATTEMPTS)
                {
                    attempts++;
                    oc_count = 0u;

                    /* Power-cycle the RP 3.3 V rail */
                    prv_rp_power_off();
                    vTaskDelay(pdMS_TO_TICKS(EPS_3V3_CYCLE_OFF_MS));
                    prv_rp_power_on();

                    /* Keep g_rp_powercycle_count in sync so boot_image_manager
                     * restarts boot monitoring and mode_state_machine's
                     * RP_POWERCYCLE_MAX_ATTEMPTS check counts OC cycles.
                     * Critical section: prevents torn RMW against rp_liveness. */
                    taskENTER_CRITICAL();
                    g_rp_powercycle_count++;
                    g_rp_live_state = RP_LIVE_OK;
                    taskEXIT_CRITICAL();

                    /* Re-init the INA219 after the rail came back */
                    (void)prv_write16(REG_CONFIG, INA219_CFG);
                    (void)prv_write16(REG_CALIB,  EPS_3V3_INA219_CALIB);
                }
                /* Beyond max attempts: fault remains set; no further
                 * cycling until current naturally drops. */
            }
        }
        else
        {
            /* Current within limits */
            oc_count = 0u;
            stable_count++;

            if ((stable_count >= EPS_3V3_STABLE_POLLS) &&
                ((g_msp_fault_bitmap & MSP_FAULT_PWR_UV_LOAD) != 0u))
            {
                fault_clear(MSP_FAULT_PWR_UV_LOAD);
                attempts     = 0u;
                stable_count = 0u;
            }
        }
    }
}

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */

void eps_3v3_monitor_task_create(void)
{
    xTaskCreate(prvEps3V3MonitorTask,
                "Eps3V3",
                EPS_3V3_STACK_SIZE,
                NULL,
                EPS_3V3_PRIORITY,
                NULL);
}
