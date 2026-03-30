/*
 * msp_status_publish.c
 *
 * MSP status publish task.
 * See msp_status_publish.h for full description.
 */

#include "msp_status_publish.h"
#include "rp_regmap.h"
#include "supervisor_i2c.h"
#include "fault_counters.h"
#include "mode_state_machine.h"
#include "battery_monitor.h"
#include "FreeRTOS.h"
#include "task.h"
#include <msp430.h>

/* ground_contact not active in this build — contact age always 0 */
static const uint32_t g_contact_age_s = 0u;

/* ------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------ */

static void prvMspStatusPublishTask(void *pvParameters)
{
    TickType_t xNextWake = xTaskGetTickCount();
    uint8_t    buf[6];   /* STATUS0, STATUS1, AGE0..AGE3 */

    (void)pvParameters;

    for (;;)
    {
        vTaskDelayUntil(&xNextWake, pdMS_TO_TICKS(MSP_STATUS_PERIOD_MS));

        /* --- Derive status inputs --- */

        /* WDT lines OK: both WDT output pins are high (not held stuck low).
         * Read P2OUT bit2 and P3OUT bit4 directly (output register reflects
         * the driven level regardless of peripheral mode). */
        uint8_t wdt_ok = ((P2OUT & BIT2) && (P3OUT & BIT4)) ? 1u : 0u;

        /* Battery OK: voltage above critical threshold */
        uint8_t batt_ok = (g_vbatt_mv >= BATT_VCRIT_MV) ? 1u : 0u;

        /* 3V3 rail OK: EN_3V3 output (P3.0) is high */
        uint8_t rail_ok = ((P3OUT & BIT0) != 0u) ? 1u : 0u;

        /* Pack STATUS0 + STATUS1 */
        uint8_t s0, s1;
        fault_pack_status((uint8_t)g_msp_mode, wdt_ok, batt_ok, rail_ok,
                          &s0, &s1);

        /* --- Write STATUS0/1 to RP regmap (0x20-0x21) --- */
        buf[0] = s0;
        buf[1] = s1;
        (void)i2c_write_reg(RP_I2C_ADDR, REG_MSP_STATUS0, buf, 2u);

        /* --- Mirror STATUS0/1 to TLM snapshot region (0xAE-0xAF) --- */
        (void)i2c_write_reg(RP_I2C_ADDR, REG_TLM_MSP_STATUS0, buf, 2u);

        /* --- Mirror authoritative contact age to RP telemetry region (0x94-0x97) --- */
        {
            uint32_t age = g_contact_age_s;
            buf[0] = (uint8_t)( age        & 0xFFu);
            buf[1] = (uint8_t)((age >>  8u) & 0xFFu);
            buf[2] = (uint8_t)((age >> 16u) & 0xFFu);
            buf[3] = (uint8_t)((age >> 24u) & 0xFFu);
            (void)i2c_write_reg(RP_I2C_ADDR, REG_TLM_CONTACT_AGE, buf, 4u);
        }

        /* --- Mirror battery voltage + current to RP telemetry (0x98-0x9B) --- */
        {
            uint16_t vb = g_vbatt_mv;
            int16_t  ib = g_ibatt_ma;
            buf[0] = (uint8_t)( vb        & 0xFFu);
            buf[1] = (uint8_t)((vb >>  8u) & 0xFFu);
            buf[2] = (uint8_t)( (uint16_t)ib        & 0xFFu);
            buf[3] = (uint8_t)(((uint16_t)ib >>  8u) & 0xFFu);
            (void)i2c_write_reg(RP_I2C_ADDR, REG_TLM_VBATT_MV, buf, 4u);
        }

        /* --- Mirror battery temp to RP telemetry (0xA2-0xA3) --- */
        {
            int16_t tb = g_tbatt_cc;
            buf[0] = (uint8_t)( (uint16_t)tb        & 0xFFu);
            buf[1] = (uint8_t)(((uint16_t)tb >>  8u) & 0xFFu);
            (void)i2c_write_reg(RP_I2C_ADDR, REG_TLM_TBATT_CC, buf, 2u);
        }

        /* --- Mirror mode to RP telemetry (0xA5) --- */
        buf[0] = (uint8_t)g_msp_mode;
        (void)i2c_write_reg(RP_I2C_ADDR, REG_TLM_MODE, buf, 1u);
    }
}

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */

void msp_status_publish_task_create(void)
{
    xTaskCreate(prvMspStatusPublishTask,
                "MspPub",
                MSP_STATUS_STACK_SIZE,
                NULL,
                MSP_STATUS_PRIORITY,
                NULL);
}
