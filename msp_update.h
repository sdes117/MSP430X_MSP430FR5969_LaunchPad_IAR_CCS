/*
 * msp_update.h
 *
 * MSP430FR firmware over-the-air (OTA) update handler.
 *
 * Receives a new MSP firmware image in chunks over CSP port
 * CSP_MSP_UPDATE, writes chunks to a staging area in FRAM, verifies
 * the CRC32 over the complete image, then triggers a POR to boot
 * the new code.
 *
 * Protocol (CSP packet payload):
 *
 *   Byte 0 : command
 *             MSP_UPD_CMD_BEGIN   (0x01) — start session, arg = image_len (u32)
 *             MSP_UPD_CMD_CHUNK   (0x02) — write chunk, arg = offset (u16) + data
 *             MSP_UPD_CMD_VERIFY  (0x03) — verify CRC32, arg = expected_crc (u32)
 *             MSP_UPD_CMD_COMMIT  (0x04) — commit + reboot
 *             MSP_UPD_CMD_ABORT   (0x05) — cancel, return to IDLE
 *             MSP_UPD_CMD_STATUS  (0x06) — query current state (read-only)
 *
 *   Response byte 0 : MSP_UPD_RESP_OK (0x00) or MSP_UPD_RESP_ERR (0x01)
 *                     followed by MSP_UPD_RESP_STATE_* byte.
 *
 * Staging area:
 *   The staging area is a fixed FRAM region defined by
 *   MSP_UPDATE_STAGING_ADDR and MSP_UPDATE_MAX_SIZE.  FRAM on the
 *   MSP430FR5969 is directly byte-writable without erase.
 *
 *   ** IMPORTANT: The staging address and size MUST be matched to
 *   the project linker script.  The default values below reserve
 *   the top 16 KB of the 64 KB FRAM space for staging.  Adjust
 *   MSP_UPDATE_STAGING_ADDR and MSP_UPDATE_MAX_SIZE to match your
 *   actual code + data footprint. **
 *
 * After COMMIT the handler:
 *   1. Calls msp_memory_scrub_invalidate() so the scrub re-baselines.
 *   2. Triggers PMM_trigPOR() to reboot into the new image.
 */

#ifndef MSP_UPDATE_H_
#define MSP_UPDATE_H_

#include <stdint.h>
#include <csp/csp.h>

/* ------------------------------------------------------------------
 * Staging area (adjust to match linker script)
 * ------------------------------------------------------------------ */
#define MSP_UPDATE_STAGING_ADDR  (0xC000u)   /* start of staging FRAM region */
#define MSP_UPDATE_MAX_SIZE      (0x3C00u)   /* 15360 bytes max image size    */

/* ------------------------------------------------------------------
 * CSP command bytes
 * ------------------------------------------------------------------ */
#define MSP_UPD_CMD_BEGIN   (0x01u)
#define MSP_UPD_CMD_CHUNK   (0x02u)
#define MSP_UPD_CMD_VERIFY  (0x03u)
#define MSP_UPD_CMD_COMMIT  (0x04u)
#define MSP_UPD_CMD_ABORT   (0x05u)
#define MSP_UPD_CMD_STATUS  (0x06u)

/* ------------------------------------------------------------------
 * Response bytes
 * ------------------------------------------------------------------ */
#define MSP_UPD_RESP_OK     (0x00u)
#define MSP_UPD_RESP_ERR    (0x01u)

/* State codes returned in response byte 1 */
#define MSP_UPD_STATE_IDLE       (0x00u)
#define MSP_UPD_STATE_RECEIVING  (0x01u)
#define MSP_UPD_STATE_VERIFIED   (0x02u)
#define MSP_UPD_STATE_ERROR      (0xFFu)

/* ------------------------------------------------------------------
 * Max chunk payload size (CSP packet body after header bytes)
 * ------------------------------------------------------------------ */
#define MSP_UPD_CHUNK_MAX_DATA   (48u)  /* bytes per chunk */

/* ------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------ */

/*
 * Handle an incoming CSP packet on the MSP_UPDATE port.
 * Called directly from prvQueueReceiveTask.
 * Sends a 2-byte response (status + state) back on the connection.
 */
void handle_csp_msp_update(csp_conn_t *conn, csp_packet_t *packet);

#endif /* MSP_UPDATE_H_ */
