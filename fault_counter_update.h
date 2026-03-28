/*
 * fault_counter_update.h
 *
 * Fault counter update task (1 Hz).
 *
 * Reads the RP-reported fault bitmap and per-fault counters from the I2C
 * regmap (0xF0-0xFD) and merges them into the MSP-local fault system:
 *
 *   - If RP fault bitmap includes I2C_LINK_ERR → set MSP_FAULT_I2C_LINK_ERR
 *   - If RP fault bitmap has any P0 faults    → log and signal mode SM
 *   - Per-counter values are stored for downlink via msp_status_publish
 *
 * Also handles fault recovery (clearing active bits when RP reports clear).
 *
 * Monitors FAULT_SEQ (0xF2) to detect when RP's fault state changes; only
 * re-reads the full counter block when SEQ advances to reduce I2C traffic.
 */

#ifndef FAULT_COUNTER_UPDATE_H_
#define FAULT_COUNTER_UPDATE_H_

#include <stdint.h>

/* Snapshot of RP-reported fault counters — read by msp_status_publish etc. */
typedef struct {
    uint16_t fault_bitmap;    /* RP fault bitmap (0xF0-0xF1) */
    uint8_t  fault_seq;       /* sequence number when snapshot was taken */
    uint8_t  latch_flags;     /* latched fault summary (0xF3) */
    uint8_t  cnt_spi_err;
    uint8_t  cnt_i2c_err;
    uint8_t  cnt_radio_tx;
    uint8_t  cnt_radio_rx_crc;
    uint8_t  cnt_mem_crc;
    uint8_t  cnt_mem_full;
    uint8_t  cnt_can_err;
    uint8_t  cnt_uart_overrun;
    uint8_t  cnt_contact_ack_to;
} rp_fault_snapshot_t;

extern volatile rp_fault_snapshot_t g_rp_fault_snapshot;

#define FAULT_UPDATE_STACK_SIZE     (configMINIMAL_STACK_SIZE + 32u)
#define FAULT_UPDATE_PRIORITY       (tskIDLE_PRIORITY + 3u)
#define FAULT_UPDATE_PERIOD_MS      (1000u)   /* 1 Hz */

void fault_counter_update_task_create(void);

#endif /* FAULT_COUNTER_UPDATE_H_ */
