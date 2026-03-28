/*
 * rp_cmd_dispatch.h
 *
 * Non-RF command dispatch and response collector.
 * Combines two task-list entries:
 *   - rp_command_dispatch  (10 Hz): write commands to RP mailbox (0x34-0x5F)
 *   - rp_response_collector(10 Hz): read RP response mailbox (0x60-0x8F)
 *
 * The command mailbox carries non-RF control messages from MSP to RP:
 *   mode-change requests, log control, config writes, non-RF resets.
 *
 * Sequence/CRC protocol:
 *   - CMD_SEQ is incremented by MSP on each new command.
 *   - CRC16-CCITT is computed over bytes [CMD_ID..CMD_PAYLOAD_END].
 *   - RP acks via RESP_SEQ matching CMD_SEQ and RESP_STATUS = OK.
 *   - MSP retries up to CMD_MAX_RETRIES times on BUSY or no ack.
 *
 * Usage:
 *   rp_cmd_submit() pushes a new command into the pending slot.
 *   The dispatch task sends it; rp_response_collector reads the ack.
 *   rp_cmd_submit() is NOT safe to call from ISR context.
 */

#ifndef RP_CMD_DISPATCH_H_
#define RP_CMD_DISPATCH_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "rp_regmap.h"

/* ------------------------------------------------------------------
 * Command IDs (non-RF only)
 * ------------------------------------------------------------------ */
#define CMD_ID_NONE             (0x00u)
#define CMD_ID_MODE_CHANGE      (0x01u)  /* payload[0] = target RP_STATE */
#define CMD_ID_LOG_CONTROL      (0x02u)  /* payload[0] = log level */
#define CMD_ID_CONFIG_WRITE     (0x03u)  /* payload = key-value pairs */
#define CMD_ID_PERIPHERAL_RESET (0x04u)  /* payload[0] = peripheral ID */
#define CMD_ID_BOOT_IMAGE       (0x05u)  /* payload[0] = BOOT_CMD_* */
#define CMD_ID_HEATER_POLICY    (0x06u)  /* payload[0] = on/off override */

/* ------------------------------------------------------------------
 * Retry and timing
 * ------------------------------------------------------------------ */
#define CMD_MAX_RETRIES         (3u)
#define CMD_RETRY_DELAY_MS      (100u)
#define CMD_DISPATCH_PERIOD_MS  (100u)   /* 10 Hz */

/* ------------------------------------------------------------------
 * Task config
 * ------------------------------------------------------------------ */
#define CMD_DISPATCH_STACK_SIZE (configMINIMAL_STACK_SIZE + 96u)
#define CMD_DISPATCH_PRIORITY   (tskIDLE_PRIORITY + 2u)

/* ------------------------------------------------------------------
 * Pending command structure
 * ------------------------------------------------------------------ */
typedef struct {
    uint8_t  cmd_id;
    uint8_t  flags;
    uint8_t  len;
    uint8_t  payload[CMD_MBOX_PAYLOAD_LEN];
} rp_cmd_t;

/* Last response received from RP. */
typedef struct {
    uint8_t  resp_id;
    uint8_t  resp_seq;
    uint8_t  resp_status;
    uint8_t  resp_len;
    uint8_t  resp_data[RESP_DATA_LEN];
} rp_resp_t;

extern volatile rp_resp_t g_last_rp_response;

/* ------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------ */

/*
 * Submit a new command to the mailbox.
 * Overwrites any pending command that has not yet been dispatched.
 * Thread-safe via a FreeRTOS mutex.
 * Returns pdTRUE if submitted, pdFALSE if mutex not available.
 */
BaseType_t rp_cmd_submit(const rp_cmd_t *cmd);

/*
 * Task creation (creates both dispatch + response collector in one task).
 */
void rp_cmd_dispatch_task_create(void);

#endif /* RP_CMD_DISPATCH_H_ */
