/*
 * event_logger.h
 *
 * FRAM-based event logger for the MSP430FR supervisor.
 * Combines two task-list entries:
 *   - msp_survival_event_logger: writes critical events when RP is off
 *   - event_log_flush (60 s): exposes log to RP when available
 *
 * Events are written to a fixed-size circular buffer in FRAM (PERSISTENT).
 * The log persists across warm resets; only POR clears PERSISTENT memory.
 *
 * Event record format (8 bytes):
 *   [0]    event_type  (EVENT_TYPE_* enum)
 *   [1]    severity    (0=P0 ... 4=INFO)
 *   [2-5]  timestamp   (g_contact_age_s at time of event, little-endian)
 *   [6]    data0       (event-specific)
 *   [7]    data1       (event-specific)
 *
 * Log capacity: EVENT_LOG_DEPTH records × 8 bytes = 512 bytes.
 * On the MSP430FR5969 with 2KB FRAM available for data, this is 25% usage.
 */

#ifndef EVENT_LOGGER_H_
#define EVENT_LOGGER_H_

#include <stdint.h>

/* ------------------------------------------------------------------
 * Event types
 * ------------------------------------------------------------------ */
#define EVENT_TYPE_NONE             (0x00u)
#define EVENT_TYPE_MODE_CHANGE      (0x01u)  /* data0=old_mode, data1=new_mode */
#define EVENT_TYPE_FAULT_SET        (0x02u)  /* data0=fault_bit_lo, data1=fault_bit_hi */
#define EVENT_TYPE_WDT_EXT_TRIP     (0x03u)  /* data0=SYSRSTIV lo, data1=hi */
#define EVENT_TYPE_RP_RESET         (0x04u)  /* data0=reset count */
#define EVENT_TYPE_RP_POWERCYCLE    (0x05u)  /* data0=cycle count */
#define EVENT_TYPE_RP_SURVIVAL      (0x06u)  /* RP entered SURVIVAL state */
#define EVENT_TYPE_CONTACT_ACCEPTED (0x07u)  /* data0=seq number */
#define EVENT_TYPE_CONTACT_REJECTED (0x08u)  /* data0=seq, data1=ack_status */
#define EVENT_TYPE_DEADMAN_RESET    (0x09u)  /* data0..3 = contact_age_s (spans data0/1) */
#define EVENT_TYPE_BOOT             (0x0Au)  /* data0=SYSRSTIV lo, data1=boot_count */
#define EVENT_TYPE_HEATER_ON        (0x0Bu)  /* data0=tbatt_cc lo, data1=hi */
#define EVENT_TYPE_HEATER_OFF       (0x0Cu)
#define EVENT_TYPE_I2C_HDR_OK       (0x0Du)  /* RP regmap header validated; data0=regmap version */
#define EVENT_TYPE_MBOX_CRC_OK      (0x0Eu)  /* Mailbox CRC pass after prior CRC failure */
#define EVENT_TYPE_OTA_BEGIN        (0x0Fu)  /* MSP OTA session started; data0=0 */
#define EVENT_TYPE_OTA_COMMIT       (0x10u)  /* MSP OTA committed; POR imminent */
#define EVENT_TYPE_OTA_ABORT        (0x11u)  /* MSP OTA aborted or CRC verify failed; data0=cmd */

/* ------------------------------------------------------------------
 * Log configuration
 * ------------------------------------------------------------------ */
#define EVENT_LOG_DEPTH         (64u)   /* number of event records */
#define EVENT_RECORD_SIZE       (8u)    /* bytes per record */

/* ------------------------------------------------------------------
 * Flush task configuration
 * ------------------------------------------------------------------ */
#define EVENT_FLUSH_STACK_SIZE  (configMINIMAL_STACK_SIZE + 64u)
#define EVENT_FLUSH_PRIORITY    (tskIDLE_PRIORITY + 1u)
#define EVENT_FLUSH_PERIOD_MS   (60000u)  /* 60 s */

/* ------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------ */

/*
 * Write one event record to the circular FRAM log.
 * Safe to call from any task context (uses a FreeRTOS mutex).
 * Does NOT block: if mutex is not immediately available, the event is dropped.
 */
void event_log_write(uint8_t event_type, uint8_t severity,
                     uint32_t timestamp,
                     uint8_t data0, uint8_t data1);

/*
 * Return the number of records currently in the log.
 */
uint16_t event_log_count(void);

/*
 * Read up to max_count records starting at offset into out_buf.
 * out_buf must be at least max_count * EVENT_RECORD_SIZE bytes.
 * Returns number of records actually copied.
 */
uint16_t event_log_read(uint16_t offset, uint8_t *out_buf, uint16_t max_count);

/*
 * Task creation for the flush task.
 */
void event_log_flush_task_create(void);

/*
 * Request an early flush without waiting for the 60 s period.
 * Safe to call from any task context.
 * Has no effect if the flush task has not been created yet.
 */
void event_log_request_flush(void);

#endif /* EVENT_LOGGER_H_ */
