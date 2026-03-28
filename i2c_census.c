/*
 * i2c_census.c
 *
 * I2C device census task.
 * See i2c_census.h for full description.
 */

#include "i2c_census.h"
#include "supervisor_i2c.h"
#include "ina219.h"
#include "mcp9808.h"
#include "rp_regmap.h"
#include "fault_counters.h"
#include "event_logger.h"
#include "ground_contact.h"
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------
 * Exported state
 * ------------------------------------------------------------------ */
volatile uint8_t g_i2c_census_bitmap = 0u;

/* ------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------ */

/* Probe a device: try to read 1 byte. Returns 1 if device ACKs. */
static uint8_t prv_probe(uint8_t addr, uint8_t reg)
{
    uint8_t buf[1] = {0u};
    return (i2c_read_reg(addr, reg, buf, 1u) == I2C_OK) ? 1u : 0u;
}

/*
 * Verify RP2350 regmap identity: reads magic bytes + version and compares
 * against the expected constants in rp_regmap.h.
 * Returns 1 if all 4 magic bytes and regmap version match, 0 otherwise.
 * Uses retry wrapper so a single transient NACK is not a false negative.
 */
static uint8_t prv_verify_rp_magic(void)
{
    uint8_t buf[5] = {0u};   /* MAGIC0..3 + REGMAP_VERSION */

    if (i2c_read_reg_retry(RP_I2C_ADDR, REG_MAGIC0, buf, 5u) != I2C_OK) {
        return 0u;
    }
    return ((buf[0] == RP_MAGIC0) &&
            (buf[1] == RP_MAGIC1) &&
            (buf[2] == RP_MAGIC2) &&
            (buf[3] == RP_MAGIC3) &&
            (buf[4] == RP_REGMAP_VERSION_EXPECTED)) ? 1u : 0u;
}

/* ------------------------------------------------------------------
 * Synchronous census poll
 * ------------------------------------------------------------------ */

uint8_t i2c_census_run(void)
{
    uint8_t bm = 0u;

    /* 1. RP2350 — validate all 4 magic bytes and regmap version */
    if (prv_verify_rp_magic())
    {
        bm |= I2C_DEV_RP2350;
    }

    /* 2. INA219 — probe its config register (0x00) */
    if (prv_probe(INA219_ADDR, 0x00u))
    {
        bm |= I2C_DEV_INA219;
    }

    /* 3. MCP9808 — probe via manufacturer ID check */
    if (mcp9808_check_id() == MCP9808_OK)
    {
        bm |= I2C_DEV_MCP9808;
    }

    g_i2c_census_bitmap = bm;

    /* Raise fault if a critical sensor is missing */
    if ((bm & I2C_DEV_INA219) == 0u)
    {
        /* Battery sensor missing — can't make voltage decisions */
        fault_set(MSP_FAULT_I2C_LINK_ERR, MSP_FCNT_I2C_LINK_ERR);
    }

    return bm;
}

/* ------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------ */

static void prvI2CCensusTask(void *pvParameters)
{
    TickType_t xNextWake    = xTaskGetTickCount();
    uint8_t    prev_bm      = 0xFFu;  /* force change-detect on first run */
    uint8_t    prev_rp_hdr  = 0u;    /* 0 = RP header not yet validated */

    (void)pvParameters;

    /* Give other tasks time to start before first census */
    vTaskDelay(pdMS_TO_TICKS(2000u));

    for (;;)
    {
        vTaskDelayUntil(&xNextWake, pdMS_TO_TICKS(I2C_CENSUS_PERIOD_MS));

        uint8_t bm = i2c_census_run();

        /* Log when RP header transitions from invalid/absent to valid */
        uint8_t rp_hdr_now = (bm & I2C_DEV_RP2350) ? 1u : 0u;
        if (rp_hdr_now && !prev_rp_hdr)
        {
            event_log_write(EVENT_TYPE_I2C_HDR_OK, 4u /* INFO */,
                            (uint32_t)g_contact_age_s,
                            RP_REGMAP_VERSION_EXPECTED, 0u);
        }
        prev_rp_hdr = rp_hdr_now;

        /* Track overall census bitmap for future change detection */
        if (bm != prev_bm)
        {
            prev_bm = bm;
        }
    }
}

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */

void i2c_census_task_create(void)
{
    xTaskCreate(prvI2CCensusTask,
                "I2CCens",
                I2C_CENSUS_STACK_SIZE,
                NULL,
                I2C_CENSUS_PRIORITY,
                NULL);
}
