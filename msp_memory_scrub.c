/*
 * msp_memory_scrub.c
 *
 * Hourly CRC32 memory scrub over the MSP430FR code FRAM region.
 * See msp_memory_scrub.h for full description.
 *
 * Uses a software CRC32 (IEEE 802.3, reflected polynomial 0xEDB88320).
 * No hardware CRC32 module on the MSP430FR5959; bit-by-bit implementation.
 */

#include "msp_memory_scrub.h"
#include "fault_counters.h"
#include "event_logger.h"
#include "ground_contact.h"
#include "driverlib.h"
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
 * Internal: software CRC32 over the scrub region (IEEE 802.3)
 * ------------------------------------------------------------------ */

static uint32_t prv_compute_crc(void)
{
    const uint8_t *ptr = (const uint8_t *)SCRUB_REGION_START_ADDR;
    uint32_t crc = 0xFFFFFFFFuL;
    uint32_t i;
    uint8_t  j;

    for (i = 0u; i < SCRUB_REGION_LEN; i++)
    {
        crc ^= (uint32_t)ptr[i];
        for (j = 0u; j < 8u; j++)
        {
            if (crc & 1uL) { crc = (crc >> 1u) ^ 0xEDB88320uL; }
            else            { crc >>= 1u; }
        }
    }
    return crc ^ 0xFFFFFFFFuL;
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
