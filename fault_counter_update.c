/*
 * fault_counter_update.c
 *
 * Reads RP fault bitmap + counters from regmap; merges into MSP fault system.
 * See fault_counter_update.h for full description.
 */

#include "fault_counter_update.h"
#include "rp_regmap.h"
#include "supervisor_i2c.h"
#include "fault_counters.h"
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------
 * Exported snapshot
 * ------------------------------------------------------------------ */
volatile rp_fault_snapshot_t g_rp_fault_snapshot = {0};

/* ------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------ */

static void prvFaultCounterUpdateTask(void *pvParameters)
{
    TickType_t xNextWake     = xTaskGetTickCount();
    uint8_t    last_seq      = 0xFFu;   /* force read on first iteration */

    (void)pvParameters;

    for (;;)
    {
        vTaskDelayUntil(&xNextWake, pdMS_TO_TICKS(FAULT_UPDATE_PERIOD_MS));

        /* -------------------------------------------------------
         * 1. Read FAULT_SEQ to check if RP fault state changed.
         * ------------------------------------------------------- */
        uint8_t seq_buf[1] = {0u};
        int8_t  rc = i2c_read_reg(RP_I2C_ADDR, REG_FAULT_SEQ, seq_buf, 1u);
        if (rc != I2C_OK)
        {
            /* I2C failure — already tracked by rp_liveness; skip this cycle */
            continue;
        }

        uint8_t rp_seq = seq_buf[0];
        if ((rp_seq == last_seq) && (last_seq != 0xFFu))
        {
            /* No change in RP fault state; skip full read */
            continue;
        }

        /* -------------------------------------------------------
         * 2. Read full fault region (0xF0-0xFC = 13 bytes).
         * ------------------------------------------------------- */
        uint8_t buf[13] = {0u};
        rc = i2c_read_reg(RP_I2C_ADDR, REG_FAULT_BITMAP_L, buf, 13u);
        if (rc != I2C_OK)
        {
            continue;
        }

        last_seq = rp_seq;

        /* Parse into snapshot */
        rp_fault_snapshot_t snap;
        snap.fault_bitmap      = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8u);
        snap.fault_seq         = buf[2];
        snap.latch_flags       = buf[3];
        snap.cnt_spi_err       = buf[4];
        snap.cnt_i2c_err       = buf[5];
        snap.cnt_radio_tx      = buf[6];
        snap.cnt_radio_rx_crc  = buf[7];
        snap.cnt_mem_crc       = buf[8];
        snap.cnt_mem_full      = buf[9];
        snap.cnt_can_err       = buf[10];
        snap.cnt_uart_overrun  = buf[11];
        snap.cnt_contact_ack_to= buf[12];

        /* Atomically update global snapshot */
        g_rp_fault_snapshot = snap;

        /* -------------------------------------------------------
         * 3. Merge RP faults into MSP fault bitmap
         * ------------------------------------------------------- */
        uint16_t rp_bm = snap.fault_bitmap;

        /* RP I2C link error → MSP I2C fault */
        if (rp_bm & RP_FAULT_I2C_LINK_ERR)
        {
            fault_set(MSP_FAULT_I2C_LINK_ERR, MSP_FCNT_I2C_LINK_ERR);
        }

        /* RP reports its I2C isolation recovery → attempt to clear */
        if ((rp_bm & RP_FAULT_I2C_ISOLATION) == 0u)
        {
            /* If our local I2C counter is zero, clear the fault bit */
            if (g_msp_fault_counts[MSP_FCNT_I2C_LINK_ERR] == 0u)
            {
                fault_clear(MSP_FAULT_I2C_LINK_ERR);
            }
        }

        /* RP permanent TX shutdown — P0 regulatory event.
         * Mirror into MSP fault bitmap so mode_state_machine escalates to SAFE
         * and the fault appears in STATUS0/1 for ground visibility. */
        if (rp_bm & RP_FAULT_TX_SHUTDOWN_PERM)
        {
            fault_set(MSP_FAULT_RP_TX_SHUTDOWN, MSP_FCNT_RP_TX_SHUTDOWN);
        }
        else
        {
            fault_clear(MSP_FAULT_RP_TX_SHUTDOWN);
        }
    }
}

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */

void fault_counter_update_task_create(void)
{
    xTaskCreate(prvFaultCounterUpdateTask,
                "FaultUpd",
                FAULT_UPDATE_STACK_SIZE,
                NULL,
                FAULT_UPDATE_PRIORITY,
                NULL);
}
