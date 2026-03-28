/*
 * msp_memory_scrub.c
 *
 * Hourly CRC32 memory scrub over the MSP430FR code FRAM region.
 * See msp_memory_scrub.h for full description.
 *
 * Uses the MSP430 hardware CRC32 module via driverlib (CRC32_MODE).
 * The hardware module is polled synchronously — no DMA or interrupt.
 * At 8 MHz, feeding 32 KB through the hardware register takes < 5 ms.
 */

#include <msp430.h>
#include "msp_memory_scrub.h"
#include "fault_counters.h"
#include "event_logger.h"
#include "ground_contact.h"
#include "driverlib.h"
#include "crc32.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

/* ------------------------------------------------------------------
 * FRAM-persistent baseline (survives warm resets; cleared on POR)
 * ------------------------------------------------------------------ */
#ifdef __ICC430__
__persistent volatile uint32_t g_scrub_crc_baseline;
__persistent volatile uint8_t  g_scrub_baseline_valid;
#else
#pragma PERSISTENT( g_scrub_crc_baseline )
volatile uint32_t g_scrub_crc_baseline = 0u;

#pragma PERSISTENT( g_scrub_baseline_valid )
volatile uint8_t  g_scrub_baseline_valid = 0u;
#endif

/* ------------------------------------------------------------------
 * API: invalidate baseline
 * ------------------------------------------------------------------ */

void msp_memory_scrub_invalidate(void)
{
    g_scrub_baseline_valid = 0u;
    g_scrub_crc_baseline   = 0u;
}

/* ------------------------------------------------------------------
 * Internal: compute CRC32 over the scrub region
 * ------------------------------------------------------------------ */

static uint32_t prv_compute_crc(void)
{
    const uint8_t *ptr = (const uint8_t *)SCRUB_REGION_START_ADDR;
    uint32_t       i;

    CRC32_setSeed(0xFFFFFFFFu, CRC32_MODE);

    for (i = 0u; i < SCRUB_REGION_LEN; i++)
    {
        CRC32_set8BitData(ptr[i], CRC32_MODE);
    }

    /* Result register returns the final CRC32 value */
    return CRC32_getResult(CRC32_MODE);
}

/* ------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------ */

static void prvMemoryScrubTask(void *pvParameters)
{
    TickType_t xNextWake = xTaskGetTickCount();

    (void)pvParameters;

    /* Delay on first boot so the system settles before the first scrub */
    vTaskDelay(pdMS_TO_TICKS(5000u));

    for (;;)
    {
        vTaskDelayUntil(&xNextWake, pdMS_TO_TICKS((uint32_t)SCRUB_INTERVAL_S * 1000u));

        uint32_t crc = prv_compute_crc();

        if (g_scrub_baseline_valid == 0u)
        {
            /* First run after POR: establish the baseline */
            g_scrub_crc_baseline   = crc;
            g_scrub_baseline_valid = 1u;
        }
        else
        {
            /* Compare against baseline */
            if (crc != g_scrub_crc_baseline)
            {
                fault_set(MSP_FAULT_MEM_CRC, MSP_FCNT_MEM_CRC);

                /* Log scrub failure: data0/1 carry low 16 bits of CRC diff */
                uint32_t diff = crc ^ g_scrub_crc_baseline;
                event_log_write(EVENT_TYPE_FAULT_SET, 1u /* P1 */,
                                (uint32_t)g_contact_age_s,
                                (uint8_t)(diff & 0xFFu),
                                (uint8_t)((diff >> 8u) & 0xFFu));
            }
            else
            {
                /* CRC matches — clear the fault if it was previously set */
                if ((g_msp_fault_bitmap & MSP_FAULT_MEM_CRC) != 0u)
                {
                    fault_clear(MSP_FAULT_MEM_CRC);
                }
            }
        }
    }
}

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */

void msp_memory_scrub_task_create(void)
{
    xTaskCreate(prvMemoryScrubTask,
                "MemScr",
                SCRUB_STACK_SIZE,
                NULL,
                SCRUB_PRIORITY,
                NULL);
}
