/*
 * msp_update.c
 *
 * MSP430FR firmware OTA update handler.
 * See msp_update.h for full description.
 */

#include <msp430.h>
#include "msp_update.h"
#include "msp_memory_scrub.h"
#include "event_logger.h"
#include "ground_contact.h"
#include "driverlib.h"
#include "crc32.h"
#include <string.h>
#include <stdint.h>

/* ------------------------------------------------------------------
 * Internal state machine
 * ------------------------------------------------------------------ */

typedef enum {
    UPD_IDLE       = 0u,
    UPD_RECEIVING  = 1u,
    UPD_VERIFIED   = 2u,
    UPD_ERROR      = 3u,
} upd_state_t;

static upd_state_t s_state     = UPD_IDLE;
static uint32_t    s_image_len = 0u;   /* expected image size from BEGIN */

/* ------------------------------------------------------------------
 * Internal: compute CRC32 over the staging region
 * ------------------------------------------------------------------ */

static uint32_t prv_crc32_staging(uint32_t len)
{
    const uint8_t *ptr = (const uint8_t *)MSP_UPDATE_STAGING_ADDR;
    uint32_t       i;

    CRC32_setSeed(0xFFFFFFFFu, CRC32_MODE);
    for (i = 0u; i < len; i++)
    {
        CRC32_set8BitData(ptr[i], CRC32_MODE);
    }
    return CRC32_getResult(CRC32_MODE);
}

/* ------------------------------------------------------------------
 * Internal: state code for response
 * ------------------------------------------------------------------ */

static uint8_t prv_state_byte(void)
{
    switch (s_state)
    {
    case UPD_IDLE:      return MSP_UPD_STATE_IDLE;
    case UPD_RECEIVING: return MSP_UPD_STATE_RECEIVING;
    case UPD_VERIFIED:  return MSP_UPD_STATE_VERIFIED;
    default:            return MSP_UPD_STATE_ERROR;
    }
}

/* ------------------------------------------------------------------
 * Internal: send 2-byte response
 * ------------------------------------------------------------------ */

static void prv_respond(csp_conn_t *conn, csp_packet_t *packet,
                        uint8_t status)
{
    packet->data[0] = status;
    packet->data[1] = prv_state_byte();
    packet->length  = 2u;
    if (!csp_send(conn, packet, 0))
    {
        csp_buffer_free(packet);
    }
}

/* ------------------------------------------------------------------
 * Public: CSP packet handler
 * ------------------------------------------------------------------ */

void handle_csp_msp_update(csp_conn_t *conn, csp_packet_t *packet)
{
    if ((packet == NULL) || (packet->length < 1u))
    {
        csp_buffer_free(packet);
        return;
    }

    uint8_t cmd = packet->data[0];

    switch (cmd)
    {
    /* -------------------------------------------------------
     * BEGIN — start a new update session
     * Payload: [0]=CMD, [1..4]=image_len (little-endian u32)
     * ------------------------------------------------------- */
    case MSP_UPD_CMD_BEGIN:
    {
        if (packet->length < 5u)
        {
            s_state = UPD_ERROR;
            prv_respond(conn, packet, MSP_UPD_RESP_ERR);
            return;
        }

        uint32_t len;
        (void)memcpy(&len, &packet->data[1], sizeof(uint32_t));

        if ((len == 0u) || (len > MSP_UPDATE_MAX_SIZE))
        {
            s_state = UPD_ERROR;
            prv_respond(conn, packet, MSP_UPD_RESP_ERR);
            return;
        }

        s_image_len = len;
        s_state     = UPD_RECEIVING;

        /* Invalidate the scrub baseline now so the hourly CRC check
         * re-baselines rather than flagging staging writes as corruption. */
        msp_memory_scrub_invalidate();

        event_log_write(EVENT_TYPE_OTA_BEGIN, 4u /* INFO */,
                        (uint32_t)g_contact_age_s, 0u, 0u);

        prv_respond(conn, packet, MSP_UPD_RESP_OK);
    }
    break;

    /* -------------------------------------------------------
     * CHUNK — write a block of bytes to staging FRAM
     * Payload: [0]=CMD, [1..2]=offset (u16 LE), [3..N]=data
     * ------------------------------------------------------- */
    case MSP_UPD_CMD_CHUNK:
    {
        if ((s_state != UPD_RECEIVING) || (packet->length < 4u))
        {
            s_state = UPD_ERROR;
            prv_respond(conn, packet, MSP_UPD_RESP_ERR);
            return;
        }

        uint16_t offset;
        (void)memcpy(&offset, &packet->data[1], sizeof(uint16_t));

        uint8_t  data_len = (uint8_t)(packet->length - 3u);

        if (((uint32_t)offset + data_len) > MSP_UPDATE_MAX_SIZE)
        {
            s_state = UPD_ERROR;
            prv_respond(conn, packet, MSP_UPD_RESP_ERR);
            return;
        }

        /* Write directly to staging FRAM — no erase required */
        uint8_t *dst = (uint8_t *)(MSP_UPDATE_STAGING_ADDR + offset);
        (void)memcpy(dst, &packet->data[3], data_len);

        prv_respond(conn, packet, MSP_UPD_RESP_OK);
    }
    break;

    /* -------------------------------------------------------
     * VERIFY — compute CRC32 over staging, compare to expected
     * Payload: [0]=CMD, [1..4]=expected_crc (u32 LE)
     * ------------------------------------------------------- */
    case MSP_UPD_CMD_VERIFY:
    {
        if ((s_state != UPD_RECEIVING) || (packet->length < 5u))
        {
            s_state = UPD_ERROR;
            prv_respond(conn, packet, MSP_UPD_RESP_ERR);
            return;
        }

        uint32_t expected_crc;
        (void)memcpy(&expected_crc, &packet->data[1], sizeof(uint32_t));

        uint32_t actual_crc = prv_crc32_staging(s_image_len);

        if (actual_crc != expected_crc)
        {
            s_state = UPD_ERROR;
            event_log_write(EVENT_TYPE_OTA_ABORT, 2u /* P2 */,
                            (uint32_t)g_contact_age_s,
                            MSP_UPD_CMD_VERIFY, 0u);
            prv_respond(conn, packet, MSP_UPD_RESP_ERR);
            return;
        }

        s_state = UPD_VERIFIED;
        prv_respond(conn, packet, MSP_UPD_RESP_OK);
    }
    break;

    /* -------------------------------------------------------
     * COMMIT — apply the verified image and reboot
     * Ground must have called VERIFY first.
     * ------------------------------------------------------- */
    case MSP_UPD_CMD_COMMIT:
    {
        if (s_state != UPD_VERIFIED)
        {
            prv_respond(conn, packet, MSP_UPD_RESP_ERR);
            return;
        }

        /* Send OK before rebooting so ground gets the ACK */
        prv_respond(conn, packet, MSP_UPD_RESP_OK);

        /* Log commit to PERSISTENT FRAM before POR — log survives warm reset
         * and will be flushed to RP when it comes up after the reboot. */
        event_log_write(EVENT_TYPE_OTA_COMMIT, 4u /* INFO */,
                        (uint32_t)g_contact_age_s, 0u, 0u);

        /* Invalidate the memory scrub baseline so it re-baselines
         * after the reboot with the new image. */
        msp_memory_scrub_invalidate();

        /* Copy staging image over the live code region in FRAM.
         * Both regions are in FRAM — direct memcpy works because
         * MSP430FR FRAM is byte-writable without prior erase.
         *
         * WARNING: This overwrites the currently executing code.
         * The copy must complete before the instruction pointer
         * reaches overwritten bytes.  This is only safe if the
         * staging and code regions do not overlap and the copy
         * proceeds from low-to-high addresses with interrupts
         * disabled (so the WDT does not trip mid-copy).
         */
        taskENTER_CRITICAL();

        (void)memcpy((void *)SCRUB_REGION_START_ADDR,
                     (const void *)MSP_UPDATE_STAGING_ADDR,
                     (size_t)s_image_len);

        /* Reboot into new code — should not return */
        PMM_trigPOR();
        for (;;) { ; }
    }
    break;

    /* -------------------------------------------------------
     * ABORT — cancel the current session
     * ------------------------------------------------------- */
    case MSP_UPD_CMD_ABORT:
        event_log_write(EVENT_TYPE_OTA_ABORT, 3u /* P3 */,
                        (uint32_t)g_contact_age_s,
                        MSP_UPD_CMD_ABORT, (uint8_t)s_state);
        s_state     = UPD_IDLE;
        s_image_len = 0u;
        prv_respond(conn, packet, MSP_UPD_RESP_OK);
        break;

    /* -------------------------------------------------------
     * STATUS — return current state (read-only)
     * ------------------------------------------------------- */
    case MSP_UPD_CMD_STATUS:
        prv_respond(conn, packet, MSP_UPD_RESP_OK);
        break;

    default:
        s_state = UPD_ERROR;
        prv_respond(conn, packet, MSP_UPD_RESP_ERR);
        break;
    }
}
