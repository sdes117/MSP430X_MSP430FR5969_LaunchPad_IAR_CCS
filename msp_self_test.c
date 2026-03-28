/*
 * msp_self_test.c
 *
 * Power-On Self Test (POST) for the MSP430FR supervisor.
 * See msp_self_test.h for full description.
 */

#include "msp_self_test.h"
#include "fault_counters.h"
#include "event_logger.h"
#include "i2c_census.h"
#include "supervisor_i2c.h"
#include "driverlib.h"
#include <msp430.h>

/* ------------------------------------------------------------------
 * Exported result
 * ------------------------------------------------------------------ */
volatile uint8_t g_post_result = 0u;

/* ------------------------------------------------------------------
 * GPIO pin definitions matching Init_GPIO() in main.c
 * ------------------------------------------------------------------ */

/* WDT output pins */
#define POST_WDT1_PORT      GPIO_PORT_P2
#define POST_WDT1_PIN       GPIO_PIN2   /* P2.2 — should be HIGH at boot */

#define POST_WDT2_PORT      GPIO_PORT_P3
#define POST_WDT2_PIN       GPIO_PIN4   /* P3.4 — should be HIGH at boot */

/* I2C pins (UCB0: P1.6 = SDA, P1.7 = SCL) — configured as peripheral,
 * but checking the physical level via GPIO input is not meaningful after
 * the pin is reassigned to the peripheral module.  Instead, we check the
 * I2C module busy flag as a proxy for bus free. */

/* ------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------ */

void msp_self_test_run(uint16_t reset_cause, uint16_t boot_count)
{
    uint8_t bm = 0u;

    /* ---------------------------------------------------------------
     * 1. WDT output lines: should be HIGH at boot (set in Init_GPIO)
     * --------------------------------------------------------------- */
    if (GPIO_getOutputPinValue(POST_WDT1_PORT, POST_WDT1_PIN) != 0u)
    {
        bm |= POST_WDT1_HIGH;
    }

    if (GPIO_getOutputPinValue(POST_WDT2_PORT, POST_WDT2_PIN) != 0u)
    {
        bm |= POST_WDT2_HIGH;
    }

    /* ---------------------------------------------------------------
     * 2. I2C bus free: attempt a dummy probe by checking if the bus
     *    can start a transaction without NACK or timeout.
     *    We probe by reading 1 byte from RP at address 0x42 register 0.
     *    Any response (ACK or NACK) means the bus is not stuck.
     *    A timeout (I2C_ERR_TIMEOUT) means the bus is stuck.
     * --------------------------------------------------------------- */
    {
        uint8_t tmp = 0u;
        int8_t  rc  = i2c_read_reg(0x42u, 0x00u, &tmp, 1u);
        if (rc != I2C_ERR_TIMEOUT)
        {
            /* Bus responded (RP may or may not be ready, but bus is free) */
            bm |= POST_I2C_BUS_FREE;
        }
    }

    /* ---------------------------------------------------------------
     * 3. I2C device census — probes INA219, MCP9808, RP2350 magic
     * --------------------------------------------------------------- */
    {
        uint8_t census = i2c_census_run();

        if (census & I2C_DEV_INA219)
        {
            bm |= POST_INA219_FOUND;
        }
        if (census & I2C_DEV_MCP9808)
        {
            bm |= POST_MCP9808_FOUND;
        }
        if (census & I2C_DEV_RP2350)
        {
            bm |= POST_RP_MAGIC_OK;
        }
    }

    /* ---------------------------------------------------------------
     * 4. Critical fault bits
     * --------------------------------------------------------------- */

    /* Missing battery sensor is a critical fault — raise immediately */
    if ((bm & POST_INA219_FOUND) == 0u)
    {
        fault_set(MSP_FAULT_I2C_LINK_ERR, MSP_FCNT_I2C_LINK_ERR);
    }

    /* External WDT reset: raise the latch fault so it is logged */
    /* SYSRSTIV_WDTKEY = 0x001C (MSP430FR) */
    if (reset_cause == 0x001Cu)
    {
        fault_set(MSP_FAULT_WDT_EXT_TRIP, MSP_FCNT_WDT_EXT_TRIP);
    }

    /* ---------------------------------------------------------------
     * 5. Store result and log boot event
     * --------------------------------------------------------------- */
    g_post_result = bm;

    /* Log boot event: data0 = SYSRSTIV low byte, data1 = boot count low byte */
    event_log_write(EVENT_TYPE_BOOT, 4u /* INFO */,
                    0u,                        /* timestamp (pre-scheduler) */
                    (uint8_t)(reset_cause & 0xFFu),
                    (uint8_t)(boot_count  & 0xFFu));
}
