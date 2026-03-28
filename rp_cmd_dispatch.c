/*
 * rp_cmd_dispatch.c
 *
 * Non-RF command mailbox dispatch + response collection.
 * See rp_cmd_dispatch.h for full description.
 *
 * CRC16: uses the hardware CRC module (CRCINIRES/CRCDI) on the MSP430FR5969
 * for speed; falls back to a SW polynomial if unavailable.
 */

#include "rp_cmd_dispatch.h"
#include "supervisor_i2c.h"
#include "fault_counters.h"
#include "event_logger.h"
#include "ground_contact.h"
#include "rp_liveness.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "driverlib.h"
#include <string.h>

/* ------------------------------------------------------------------
 * CRC16-CCITT (0x1021 polynomial) — software implementation.
 * Used to protect the command mailbox payload.
 * ------------------------------------------------------------------ */
static uint16_t prv_crc16(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFFu;
    uint8_t  i;
    uint8_t  j;

    for (i = 0u; i < len; i++)
    {
        crc ^= ((uint16_t)data[i] << 8u);
        for (j = 0u; j < 8u; j++)
        {
            if (crc & 0x8000u)
            {
                crc = (uint16_t)((crc << 1u) ^ 0x1021u);
            }
            else
            {
                crc <<= 1u;
            }
        }
    }
    return crc;
}

/* ------------------------------------------------------------------
 * Exported state
 * ------------------------------------------------------------------ */
volatile rp_resp_t g_last_rp_response = {0};

/* ------------------------------------------------------------------
 * Internal state
 * ------------------------------------------------------------------ */
static SemaphoreHandle_t xCmdMutex    = NULL;
static rp_cmd_t          xPendingCmd  = {0};
static uint8_t           bCmdPending  = 0u;
static uint8_t           g_cmd_seq    = 0u;  /* monotonic mailbox sequence */

/* ------------------------------------------------------------------
 * API: submit command
 * ------------------------------------------------------------------ */

BaseType_t rp_cmd_submit(const rp_cmd_t *cmd)
{
    if (xSemaphoreTake(xCmdMutex, pdMS_TO_TICKS(10u)) != pdTRUE)
    {
        return pdFALSE;
    }

    xPendingCmd = *cmd;
    bCmdPending = 1u;

    xSemaphoreGive(xCmdMutex);
    return pdTRUE;
}

/* ------------------------------------------------------------------
 * Internal: write command to RP mailbox
 * ------------------------------------------------------------------ */

static int8_t prv_write_command(const rp_cmd_t *cmd, uint8_t seq)
{
    uint8_t  buf[4u + CMD_MBOX_PAYLOAD_LEN + 2u];  /* header + payload + CRC */
    uint16_t crc;
    uint8_t  total;

    buf[0] = cmd->cmd_id;
    buf[1] = seq;
    buf[2] = cmd->len;
    buf[3] = cmd->flags;

    if (cmd->len > 0u)
    {
        (void)memcpy(&buf[4], cmd->payload, (size_t)cmd->len);
    }

    /* Zero-pad unused payload bytes */
    if (cmd->len < CMD_MBOX_PAYLOAD_LEN)
    {
        (void)memset(&buf[4u + cmd->len], 0, CMD_MBOX_PAYLOAD_LEN - cmd->len);
    }

    /* CRC over all bytes from CMD_ID to end of payload */
    total = 4u + CMD_MBOX_PAYLOAD_LEN;
    crc   = prv_crc16(buf, total);
    buf[total]     = (uint8_t)(crc & 0xFFu);   /* CRC_L */
    buf[total + 1u] = (uint8_t)(crc >> 8u);    /* CRC_H */

    return i2c_write_reg(RP_I2C_ADDR, REG_CMD_MBOX_ID,
                         buf, total + 2u);
}

/* ------------------------------------------------------------------
 * Internal: read response mailbox
 * ------------------------------------------------------------------ */

static int8_t prv_read_response(rp_resp_t *resp_out)
{
    uint8_t  buf[4u + RESP_DATA_LEN + 2u];
    uint16_t crc_calc;
    uint16_t crc_recv;
    int8_t   rc;

    rc = i2c_read_reg(RP_I2C_ADDR, REG_RESP_ID, buf, 4u + RESP_DATA_LEN + 2u);
    if (rc != I2C_OK)
    {
        return rc;
    }

    /* Verify CRC */
    crc_calc = prv_crc16(buf, 4u + RESP_DATA_LEN);
    crc_recv = (uint16_t)buf[4u + RESP_DATA_LEN] |
               ((uint16_t)buf[4u + RESP_DATA_LEN + 1u] << 8u);

    if (crc_calc != crc_recv)
    {
        return I2C_ERR_NACK;  /* repurpose as CRC error indicator */
    }

    resp_out->resp_id     = buf[0];
    resp_out->resp_seq    = buf[1];
    resp_out->resp_status = buf[2];
    resp_out->resp_len    = buf[3];
    (void)memcpy((void *)resp_out->resp_data, &buf[4], RESP_DATA_LEN);

    return I2C_OK;
}

/* ------------------------------------------------------------------
 * Task: combined dispatch + response collection
 * ------------------------------------------------------------------ */

static void prvCmdDispatchTask(void *pvParameters)
{
    TickType_t xNextWake      = xTaskGetTickCount();
    uint8_t    retry          = 0u;
    uint8_t    sent_seq       = 0u;
    uint8_t    active         = 0u;   /* 1 = command in flight waiting for ack */
    uint8_t    crc_failed_last = 0u;  /* 1 = last response read had a CRC error */

    (void)pvParameters;

    for (;;)
    {
        vTaskDelayUntil(&xNextWake, pdMS_TO_TICKS(CMD_DISPATCH_PERIOD_MS));

        /* -------------------------------------------------------
         * A. Read RP response mailbox
         * ------------------------------------------------------- */
        {
            rp_resp_t resp;
            int8_t    resp_rc = prv_read_response(&resp);

            if (resp_rc == I2C_OK)
            {
                /* Log CRC pass on transition from a prior CRC failure */
                if (crc_failed_last)
                {
                    crc_failed_last = 0u;
                    event_log_write(EVENT_TYPE_MBOX_CRC_OK, 4u /* INFO */,
                                    (uint32_t)g_contact_age_s, 0u, 0u);
                }

                if (active && (resp.resp_seq == sent_seq))
                {
                    /* Got ack for our last command */
                    g_last_rp_response = resp;
                    active = 0u;
                    retry  = 0u;
                }
            }
            else
            {
                /* Mark CRC/I2C failure for transition detection */
                crc_failed_last = 1u;
            }
        }

        /* -------------------------------------------------------
         * B. Send pending command (or retry)
         * ------------------------------------------------------- */
        if (xSemaphoreTake(xCmdMutex, 0u) == pdTRUE)
        {
            if (bCmdPending && !active)
            {
                /* New command: assign sequence */
                g_cmd_seq++;
                sent_seq   = g_cmd_seq;
                retry      = 0u;
                active     = 1u;
                bCmdPending = 0u;

                (void)prv_write_command(&xPendingCmd, sent_seq);
            }
            xSemaphoreGive(xCmdMutex);
        }

        /* Retry logic: if command in-flight and no ack yet */
        if (active)
        {
            retry++;
            if (retry >= (CMD_MAX_RETRIES * (CMD_RETRY_DELAY_MS / CMD_DISPATCH_PERIOD_MS)))
            {
                /* Retry limit reached — abandon this command */
                active = 0u;
                retry  = 0u;
                fault_set(MSP_FAULT_I2C_LINK_ERR, MSP_FCNT_I2C_LINK_ERR);
            }
        }

        /* -------------------------------------------------------
         * C. Read and dispatch RP request flags (0x30-0x33)
         *
         * RP sets REQ_PENDING to ask MSP for supervisor-only actions
         * it cannot perform itself (power-cycle, fault clear, etc.).
         * Flags are sticky — they persist until MSP clears them.
         * Running at 10 Hz means worst-case latency is 100 ms.
         * ------------------------------------------------------- */
        {
            uint8_t req_buf[4] = {0u};

            if (i2c_read_reg(RP_I2C_ADDR, REG_RP_REQ_FLAGS,
                             req_buf, 4u) == I2C_OK)
            {
                uint8_t flags = req_buf[0];  /* REG_RP_REQ_FLAGS */
                uint8_t code  = req_buf[1];  /* REG_RP_REQ_CODE  */
                uint8_t arg0  = req_buf[2];  /* REG_RP_REQ_ARG0  */
                uint8_t arg1  = req_buf[3];  /* REG_RP_REQ_ARG1  */

                (void)arg0;
                (void)arg1;

                if (flags & RP_REQ_PENDING)
                {
                    switch (code)
                    {
                    case RP_REQ_POWER_CYCLE_RP:
                        /* RP is requesting its own power-cycle (e.g. after
                         * a self-diagnosed unrecoverable fault).  Drive the
                         * liveness escalation directly to the power-cycle
                         * rung — rp_liveness will handle the EN_3V3 timing. */
                        g_rp_live_state = RP_LIVE_POWERCYCLE;
                        event_log_write(EVENT_TYPE_RP_POWERCYCLE, 1u /* P1 */,
                                        (uint32_t)g_contact_age_s,
                                        g_rp_powercycle_count, code);
                        break;

                    case RP_REQ_CLEAR_FAULT_COUNTERS:
                        /* RP housekeeping: clear the non-latching MSP counters
                         * that RP contributes to (I2C link, load rail). */
                        fault_clear(MSP_FAULT_I2C_LINK_ERR);
                        fault_clear(MSP_FAULT_PWR_UV_LOAD);
                        break;

                    case RP_REQ_HEATER_POLICY_REFRESH:
                        /* battery_monitor already re-evaluates g_heater_permitted
                         * every poll; no explicit action needed here. */
                        break;

                    case RP_REQ_SAFE_MODE_CONFIRM:
                        /* RP confirms it has entered SAFE mode on its own initiative.
                         * Log the event; mode_state_machine will pick up the RP
                         * state register independently on its next poll. */
                        event_log_write(EVENT_TYPE_MODE_CHANGE, 1u /* P1 */,
                                        (uint32_t)g_contact_age_s,
                                        (uint8_t)0xFFu,  /* old = unknown from RP side */
                                        RP_STATE_SAFE);
                        break;

                    case RP_REQ_CONFIG_COMMIT:
                        /* RP asking MSP to acknowledge a config change it applied.
                         * No MSP-local action needed; log for ground visibility. */
                        event_log_write(EVENT_TYPE_FAULT_SET, 4u /* INFO */,
                                        (uint32_t)g_contact_age_s,
                                        code, arg0);
                        break;

                    case RP_REQ_TIME_SYNC:
                        /* RP has a better time estimate.  The full Unix epoch
                         * requires 4 bytes; only 2 args are available here, so
                         * a meaningful set is not possible via this path.
                         * Log the hint; a TC-initiated time-set via CSP is the
                         * authoritative mechanism (handle_csp_timesync). */
                        event_log_write(EVENT_TYPE_FAULT_SET, 4u /* INFO */,
                                        (uint32_t)g_contact_age_s,
                                        code, 0u);
                        break;

                    case RP_REQ_EVENT_LOG_SNAPSHOT:
                        /* RP wants the event log flushed now (e.g. before a
                         * planned shutdown).  Unblock the flush task early. */
                        event_log_request_flush();
                        break;

                    case RP_REQ_BOOT_STATUS_REFRESH:
                        /* boot_image_manager monitors RP state register
                         * independently; this is a no-op from MSP's side. */
                        break;

                    default:
                        /* Unknown request code — log it so ground can diagnose
                         * a regmap version mismatch. */
                        event_log_write(EVENT_TYPE_FAULT_SET, 3u /* P3 */,
                                        (uint32_t)g_contact_age_s,
                                        code, flags);
                        break;
                    }

                    /* Acknowledge: write 0 to REG_RP_REQ_FLAGS to clear the
                     * pending bit.  Only do this when RP set CLEAR_AFTER_ACK;
                     * otherwise leave it for RP to clear on its side. */
                    if (flags & RP_REQ_CLEAR_AFTER_ACK)
                    {
                        uint8_t clear = 0x00u;
                        (void)i2c_write_reg(RP_I2C_ADDR, REG_RP_REQ_FLAGS,
                                            &clear, 1u);
                    }
                }
            }
        }
    }
}

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */

void rp_cmd_dispatch_task_create(void)
{
    xCmdMutex = xSemaphoreCreateMutex();
    configASSERT(xCmdMutex != NULL);

    xTaskCreate(prvCmdDispatchTask,
                "CmdDsp",
                CMD_DISPATCH_STACK_SIZE,
                NULL,
                CMD_DISPATCH_PRIORITY,
                NULL);
}
