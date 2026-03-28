/*
 * ground_contact.c
 *
 * Ground contact processor + 48-hour deadman timer.
 * See ground_contact.h for full description.
 */

#include "ground_contact.h"
#include "rp_regmap.h"
#include "supervisor_i2c.h"
#include "fault_counters.h"
#include "rp_liveness.h"
#include "FreeRTOS.h"
#include "task.h"
#include "driverlib.h"
#include <msp430.h>

/* ------------------------------------------------------------------
 * FRAM-persistent state (NOINIT — survives warm reset, not POR)
 * ------------------------------------------------------------------ */
#ifdef __ICC430__
__no_init volatile uint32_t g_contact_age_s;
__no_init volatile uint8_t  g_last_contact_seq;
#else
#pragma NOINIT(g_contact_age_s)
volatile uint32_t g_contact_age_s;
#pragma NOINIT(g_last_contact_seq)
volatile uint8_t  g_last_contact_seq;
#endif

/* ------------------------------------------------------------------
 * 1 Hz tick — called from clock task
 * ------------------------------------------------------------------ */

void ground_contact_tick_1hz(void)
{
    g_contact_age_s++;

    if (g_contact_age_s >= DEADMAN_TIMEOUT_S)
    {
        if (g_rp_live_state == RP_LIVE_I2C_STALL)
        {
            /*
             * I2C dead but RP GPIO heartbeat still active: the deadman fired
             * because MSP cannot receive contact events over I2C, not because
             * RP has stopped operating.  Attempt an RP hard reset to recover
             * the I2C link; postpone the MSP POR deadline by
             * DEADMAN_RP_RESET_PERIOD_S.  If RP eventually loses its GPIO
             * heartbeat too (escalates past RP_LIVE_I2C_STALL), the next
             * tick takes the POR branch below.
             */
            g_contact_age_s -= DEADMAN_RP_RESET_PERIOD_S;
            g_rp_live_state  = RP_LIVE_GPIO_STALL;  /* force RP reset via liveness task */
        }
        else
        {
            /* Genuine no-contact for 48 h, or RP fully dead: MSP full POR
             * per GDIR-MIS-12.  Should not return. */
            PMM_trigPOR();
            for (;;) { ; }
        }
    }
}

/* ------------------------------------------------------------------
 * Helper: write contact acknowledge to RP regmap
 * ------------------------------------------------------------------ */

static void prvWriteContactAck(uint8_t ack_seq, uint8_t ack_status,
                               uint32_t contact_age_s)
{
    uint8_t buf[6];

    buf[0] = ack_seq;
    buf[1] = ack_status;
    buf[2] = (uint8_t)( contact_age_s        & 0xFFu);
    buf[3] = (uint8_t)((contact_age_s >>  8u) & 0xFFu);
    buf[4] = (uint8_t)((contact_age_s >> 16u) & 0xFFu);
    buf[5] = (uint8_t)((contact_age_s >> 24u) & 0xFFu);

    (void)i2c_write_reg(RP_I2C_ADDR, REG_CONTACT_ACK_SEQ, buf, 6u);
}

/* ------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------ */

static void prvGroundContactTask(void *pvParameters)
{
    TickType_t xNextWake = xTaskGetTickCount();
    uint8_t    buf[8];
    int8_t     rc;

    (void)pvParameters;

    for (;;)
    {
        vTaskDelayUntil(&xNextWake, pdMS_TO_TICKS(GROUND_CONTACT_PERIOD_MS));

        /* -------------------------------------------------------
         * SEQ-guarded read of the contact event block (8 bytes: 0x18-0x1F).
         *
         * RP may update the block mid-read (torn read).  Guard against this
         * by reading CONTACT_EVT_SEQ before and after the full block read:
         * if the byte changed, RP wrote a new event during our read and the
         * data may be inconsistent — retry up to CONTACT_READ_MAX_RETRIES.
         * ------------------------------------------------------- */
        {
            uint8_t seq_before = 0u;
            uint8_t seq_after  = 0u;
            uint8_t tries      = 0u;

            do {
                rc = i2c_read_reg_retry(RP_I2C_ADDR, REG_CONTACT_EVT_SEQ,
                                        &seq_before, 1u);
                if (rc != I2C_OK) { break; }

                rc = i2c_read_reg_retry(RP_I2C_ADDR, REG_CONTACT_EVT_SEQ,
                                        buf, 8u);
                if (rc != I2C_OK) { break; }

                rc = i2c_read_reg_retry(RP_I2C_ADDR, REG_CONTACT_EVT_SEQ,
                                        &seq_after, 1u);
                if (rc != I2C_OK) { break; }

                tries++;
            } while ((seq_before != seq_after) &&
                     (tries < CONTACT_READ_MAX_RETRIES));
        }
        if (rc != I2C_OK)
        {
            /* I2C read failed — not a contact rejection, just skip */
            continue;
        }

        uint8_t  evt_seq     = buf[0];  /* CONTACT_EVT_SEQ     */
        uint8_t  evt_type    = buf[1];  /* CONTACT_EVT_TYPE    */
        uint8_t  evt_flags   = buf[2];  /* CONTACT_EVT_FLAGS   */
        uint8_t  evt_pending = buf[3];  /* CONTACT_EVT_PENDING */
        /* bytes [4:7] = CONTACT_EVT_TIME (unused in policy check) */

        if (evt_pending == 0u)
        {
            /* No pending event — nothing to do */
            continue;
        }

        /* Duplicate check: already processed this sequence number? */
        if (evt_seq == g_last_contact_seq)
        {
            /* Duplicate — re-send previous ack with current age */
            prvWriteContactAck(g_last_contact_seq,
                               CONTACT_ACK_ACCEPTED,
                               g_contact_age_s);
            continue;
        }

        /* -------------------------------------------------------
         * Validation policy
         * ------------------------------------------------------- */
        uint8_t ack_status = CONTACT_ACK_ACCEPTED;

        /* Type must be a known valid contact event */
        if ((evt_type < CONTACT_TYPE_VALID_TC_UPLINK) ||
            (evt_type > CONTACT_TYPE_GROUND_KEEPALIVE))
        {
            ack_status = CONTACT_ACK_REJECTED_INVTYPE;
        }
        /* Required authentication + CRC flags must be set */
        else if ((evt_flags & CONTACT_FLAGS_VALID_MIN) != CONTACT_FLAGS_VALID_MIN)
        {
            ack_status = CONTACT_ACK_REJECTED_BAD_FLAGS;
        }

        /* -------------------------------------------------------
         * Accept or reject
         * ------------------------------------------------------- */
        if (ack_status == CONTACT_ACK_ACCEPTED)
        {
            /* Reset authoritative contact age */
            g_contact_age_s    = 0u;
            g_last_contact_seq = evt_seq;

            prvWriteContactAck(evt_seq, CONTACT_ACK_ACCEPTED, 0u);
        }
        else
        {
            prvWriteContactAck(evt_seq, ack_status, g_contact_age_s);
        }
    }
}

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */

void ground_contact_task_create(void)
{
    xTaskCreate(prvGroundContactTask,
                "GndCont",
                GROUND_CONTACT_STACK_SIZE,
                NULL,
                GROUND_CONTACT_PRIORITY,
                NULL);
}
