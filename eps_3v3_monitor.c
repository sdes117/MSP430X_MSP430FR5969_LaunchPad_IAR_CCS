/*
 * eps_3v3_monitor.c
 *
 * MSP 3.3 V supply current monitor — observe only, no rail cycling.
 * See eps_3v3_monitor.h for full description.
 */

#include "eps_3v3_monitor.h"
#include "supervisor_i2c.h"
#include "fault_counters.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

/* INA219 register addresses */
#define REG_CONFIG   (0x00u)
#define REG_CURRENT  (0x04u)
#define REG_CALIB    (0x05u)

/* 32V range, /8 gain, 12-bit, continuous */
#define INA219_CFG   (0x3FFFu)

static int8_t prv_write16(uint8_t reg, uint16_t val)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(val >> 8u);
    buf[1] = (uint8_t)(val & 0xFFu);
    return i2c_write_reg(EPS_3V3_INA219_ADDR, reg, buf, 2u);
}

static int8_t prv_read_current_ma(int16_t *current_ma_out)
{
    uint8_t buf[2] = {0u, 0u};
    int8_t  rc = i2c_read_reg(EPS_3V3_INA219_ADDR, REG_CURRENT, buf, 2u);
    if (rc != I2C_OK) { return rc; }

    int16_t raw = (int16_t)(((uint16_t)buf[0] << 8u) | (uint16_t)buf[1]);
    int32_t ua  = (int32_t)raw * (int32_t)EPS_3V3_INA219_CURRENT_LSB_UA;
    *current_ma_out = (int16_t)(ua / 1000);
    return I2C_OK;
}

static void prvEps3V3MonitorTask(void *pvParameters)
{
    TickType_t xNextWake    = xTaskGetTickCount();
    uint8_t    oc_count     = 0u;
    uint8_t    stable_count = 0u;

    (void)pvParameters;

    (void)prv_write16(REG_CONFIG, INA219_CFG);
    (void)prv_write16(REG_CALIB,  EPS_3V3_INA219_CALIB);

    for (;;)
    {
        vTaskDelayUntil(&xNextWake, pdMS_TO_TICKS(EPS_3V3_POLL_MS));

        int16_t current_ma = 0;
        if (prv_read_current_ma(&current_ma) != I2C_OK)
        {
            oc_count = 0u;
            continue;
        }

        if (current_ma > (int16_t)EPS_3V3_OC_THRESHOLD_MA)
        {
            stable_count = 0u;
            if (++oc_count >= EPS_3V3_OC_TRIP_THRESHOLD)
            {
                /* eFuse hardware limits the current autonomously.
                 * MSP must NOT cycle its own supply — just flag it. */
                fault_set(MSP_FAULT_PWR_OC_MCU, MSP_FCNT_PWR_OC_MCU);
            }
        }
        else
        {
            oc_count = 0u;
            if (++stable_count >= EPS_3V3_STABLE_POLLS)
            {
                fault_clear(MSP_FAULT_PWR_OC_MCU);
                stable_count = 0u;
            }
        }
    }
}

void eps_3v3_monitor_task_create(void)
{
    xTaskCreate(prvEps3V3MonitorTask,
                "Eps3V3",
                EPS_3V3_STACK_SIZE,
                NULL,
                EPS_3V3_PRIORITY,
                NULL);
}
