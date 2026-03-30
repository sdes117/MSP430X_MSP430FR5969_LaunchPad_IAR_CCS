/*
 * event_logger.c
 *
 * FRAM circular event log + flush task.
 * See event_logger.h for full description.
 */

#include "event_logger.h"
#include "rp_cmd_dispatch.h"
#include "ground_contact.h"
#include "mode_state_machine.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

/* ------------------------------------------------------------------
 * FRAM-persistent circular log
 * The buffer, head, and count are all PERSISTENT so they survive warm resets.
 * ------------------------------------------------------------------ */
#ifdef __ICC430__
__persistent static uint8_t  xLogBuf[EVENT_LOG_DEPTH][EVENT_RECORD_SIZE];
__persistent static uint16_t xLogHead  = 0u;
__persistent static uint16_t xLogCount = 0u;
#else
#pragma PERSISTENT(xLogBuf)
static uint8_t  xLogBuf[EVENT_LOG_DEPTH][EVENT_RECORD_SIZE] = {{0u}};

#pragma PERSISTENT(xLogHead)
static uint16_t xLogHead  = 0u;

#pragma PERSISTENT(xLogCount)
static uint16_t xLogCount = 0u;
#endif

/* Mutex protecting log state */
static SemaphoreHandle_t xLogMutex      = NULL;

/* Task handle for early-flush notifications */
static TaskHandle_t      xFlushTask     = NULL;

/* ------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------ */

void event_log_write(uint8_t event_type, uint8_t severity,
                     uint32_t timestamp,
                     uint8_t data0, uint8_t data1)
{
    uint8_t  rec[EVENT_RECORD_SIZE];
    uint16_t slot;

    rec[0] = event_type;
    rec[1] = severity;
    rec[2] = (uint8_t)( timestamp        & 0xFFu);
    rec[3] = (uint8_t)((timestamp >>  8u) & 0xFFu);
    rec[4] = (uint8_t)((timestamp >> 16u) & 0xFFu);
    rec[5] = (uint8_t)((timestamp >> 24u) & 0xFFu);
    rec[6] = data0;
    rec[7] = data1;

    if (xLogMutex == NULL)
    {
        /* Mutex not created yet (early boot or wrong call order) — skip. */
        return;
    }

    if (xSemaphoreTake(xLogMutex, 0u) != pdTRUE)
    {
        /* Drop event rather than block */
        return;
    }

    /* Write to head position (overwrites oldest if full — circular) */
    slot = xLogHead % EVENT_LOG_DEPTH;
    (void)memcpy(xLogBuf[slot], rec, EVENT_RECORD_SIZE);

    xLogHead = (uint16_t)((xLogHead + 1u) % EVENT_LOG_DEPTH);

    if (xLogCount < EVENT_LOG_DEPTH)
    {
        xLogCount++;
    }

    xSemaphoreGive(xLogMutex);
}

uint16_t event_log_count(void)
{
    return xLogCount;
}

uint16_t event_log_read(uint16_t offset, uint8_t *out_buf, uint16_t max_count)
{
    uint16_t n       = 0u;
    uint16_t total   = xLogCount;
    uint16_t start;

    if (offset >= total)
    {
        return 0u;
    }

    if (max_count > (total - offset))
    {
        max_count = total - offset;
    }

    /*
     * Records are stored oldest-first in a circular buffer.
     * Oldest record is at (xLogHead - xLogCount + offset) mod DEPTH
     * if the buffer is full; or at offset if not full.
     */
    if (xLogCount < EVENT_LOG_DEPTH)
    {
        start = offset;
    }
    else
    {
        start = (uint16_t)((xLogHead + offset) % EVENT_LOG_DEPTH);
    }

    for (n = 0u; n < max_count; n++)
    {
        uint16_t idx = (uint16_t)((start + n) % EVENT_LOG_DEPTH);
        (void)memcpy(&out_buf[n * EVENT_RECORD_SIZE],
                     xLogBuf[idx],
                     EVENT_RECORD_SIZE);
    }

    return max_count;
}

/* ------------------------------------------------------------------
 * Early-flush request: unblock the flush task immediately
 * ------------------------------------------------------------------ */

void event_log_request_flush(void)
{
    if (xFlushTask != NULL)
    {
        xTaskNotifyGive(xFlushTask);
    }
}

/* ------------------------------------------------------------------
 * Early init: create the mutex before the scheduler starts so that
 * event_log_write() calls from msp_self_test_run() are not dropped.
 * ------------------------------------------------------------------ */

void event_log_init(void)
{
    if (xLogMutex == NULL)
    {
        xLogMutex = xSemaphoreCreateMutex();
        configASSERT(xLogMutex != NULL);
    }
}

/* ------------------------------------------------------------------
 * Flush task: push recent log entries to RP via command mailbox
 * ------------------------------------------------------------------ */

static void prvEventLogFlushTask(void *pvParameters)
{
    uint16_t last_flushed = 0u;

    (void)pvParameters;

    /* Create mutex if event_log_init() was not called before the scheduler */
    if (xLogMutex == NULL)
    {
        xLogMutex = xSemaphoreCreateMutex();
        configASSERT(xLogMutex != NULL);
    }

    for (;;)
    {
        /* Wait up to EVENT_FLUSH_PERIOD_MS, or until notified early by
         * event_log_request_flush() (e.g. from RP_REQ_EVENT_LOG_SNAPSHOT). */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(EVENT_FLUSH_PERIOD_MS));

        uint16_t total = xLogCount;
        if (total <= last_flushed)
        {
            last_flushed = total;  /* reset if log wrapped */
            continue;
        }

        /* Send up to CMD_MBOX_PAYLOAD_LEN / EVENT_RECORD_SIZE records per flush */
        uint16_t to_send = total - last_flushed;
        uint16_t batch   = CMD_MBOX_PAYLOAD_LEN / EVENT_RECORD_SIZE;  /* 4 records */

        if (to_send > batch)
        {
            to_send = batch;
        }

        rp_cmd_t cmd;
        (void)memset(&cmd, 0, sizeof(cmd));
        cmd.cmd_id = CMD_ID_LOG_CONTROL;
        cmd.len    = (uint8_t)(to_send * EVENT_RECORD_SIZE);

        (void)event_log_read(last_flushed, cmd.payload, to_send);
        (void)rp_cmd_submit(&cmd);

        last_flushed += to_send;
    }
}

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */

void event_log_flush_task_create(void)
{
    xTaskCreate(prvEventLogFlushTask,
                "EvtFlush",
                EVENT_FLUSH_STACK_SIZE,
                NULL,
                EVENT_FLUSH_PRIORITY,
                &xFlushTask);
}
