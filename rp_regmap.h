/*
 * rp_regmap.h
 *
 * MSP430FR (master) <-> RP2350 (slave) I2C register map.
 * 256-byte address space. All multi-byte fields are little-endian.
 *
 * Enum values sourced from HIA_Task_Schedule_MSP_Master_RP_Slave_v4.xlsx,
 * "MSP<->RP Register Map" sheet annotations.
 *
 * Direction annotations:
 *   M->R  MSP writes, RP reads
 *   R->M  RP writes, MSP reads
 */

#ifndef RP_REGMAP_H_
#define RP_REGMAP_H_

#include <stdint.h>

/* RP2350 I2C slave address (7-bit). Must match RP firmware. */
#define RP_I2C_ADDR                 (0x42u)

/* Magic bytes to validate correct device / regmap version. */
#define RP_MAGIC0                   (0x48u)  /* 'H' */
#define RP_MAGIC1                   (0x49u)  /* 'I' */
#define RP_MAGIC2                   (0x41u)  /* 'A' */
#define RP_MAGIC3                   (0x31u)  /* '1' */
#define RP_REGMAP_VERSION_EXPECTED  (0x01u)

/* ======================================================================
 * Register addresses
 * ====================================================================== */

/* ---------- Region: Header (0x00-0x0F, R->M at boot for validation) ------- */
#define REG_MAGIC0          (0x00u)
#define REG_MAGIC1          (0x01u)
#define REG_MAGIC2          (0x02u)
#define REG_MAGIC3          (0x03u)
#define REG_REGMAP_VERSION  (0x04u)
#define REG_REGMAP_LEN_L    (0x05u)
#define REG_REGMAP_LEN_H    (0x06u)
#define REG_ENDIAN          (0x07u)
#define REG_BUILD_ID0       (0x08u)
#define REG_BUILD_ID1       (0x09u)
#define REG_BUILD_ID2       (0x0Au)
#define REG_BUILD_ID3       (0x0Bu)
#define REG_I2C_ADDR        (0x0Cu)
#define REG_FLAGS           (0x0Du)
#define REG_HDR_CRC_L       (0x0Eu)
#define REG_HDR_CRC_H       (0x0Fu)

/* ---------- Region: RP State/Health (0x10-0x17, R->M) --------------------- */
#define REG_RP_STATE        (0x10u)
#define REG_UPTIME0         (0x11u)  /* uptime_s byte 0 (LSB) */
#define REG_UPTIME1         (0x12u)
#define REG_UPTIME2         (0x13u)
#define REG_UPTIME3         (0x14u)  /* MSB */
#define REG_HB0             (0x15u)  /* heartbeat counter LSB */
#define REG_HB1             (0x16u)  /* heartbeat counter MSB */
#define REG_LAST_ERROR      (0x17u)

/* ---------- Region: Ground Contact Candidate (0x18-0x1F, R->M) ------------ */
#define REG_CONTACT_EVT_SEQ     (0x18u)
#define REG_CONTACT_EVT_TYPE    (0x19u)
#define REG_CONTACT_EVT_FLAGS   (0x1Au)
#define REG_CONTACT_EVT_PENDING (0x1Bu)  /* 1=pending/unacked, 0=none */
#define REG_CONTACT_EVT_TIME0   (0x1Cu)
#define REG_CONTACT_EVT_TIME1   (0x1Du)
#define REG_CONTACT_EVT_TIME2   (0x1Eu)
#define REG_CONTACT_EVT_TIME3   (0x1Fu)

/* ---------- Region: MSP Status Bytes (0x20-0x21, M->R) -------------------- */
#define REG_MSP_STATUS0     (0x20u)
#define REG_MSP_STATUS1     (0x21u)

/* ---------- Region: Ground Contact Ack/Timer (0x22-0x27, M->R) ------------ */
#define REG_CONTACT_ACK_SEQ     (0x22u)
#define REG_CONTACT_ACK_STATUS  (0x23u)
#define REG_LAST_CONTACT_AGE0   (0x24u)  /* seconds since last accepted contact, byte 0 */
#define REG_LAST_CONTACT_AGE1   (0x25u)
#define REG_LAST_CONTACT_AGE2   (0x26u)
#define REG_LAST_CONTACT_AGE3   (0x27u)

/* ---------- Region: Mode/Power Commands (0x28-0x2F, M->R except CMD_STATUS) */
#define REG_MODE_CMD        (0x28u)  /* MSP-requested RP operating mode */
#define REG_POWER_CMD       (0x29u)  /* Power action; works with RAIL_MASK_LO/HI */
#define REG_RESET_CMD       (0x2Au)  /* Non-power-cut reset / local recovery */
#define REG_BOOT_CMD        (0x2Bu)  /* Boot image / boot behaviour */
#define REG_RAIL_MASK_LO    (0x2Cu)  /* Rail allow mask bits [7:0] */
#define REG_RAIL_MASK_HI    (0x2Du)  /* Rail allow mask bits [15:8] */
#define REG_CMD_SEQ         (0x2Eu)  /* Monotonic command sequence (MSP increments) */
#define REG_CMD_STATUS      (0x2Fu)  /* RP ack bitfield (R->M) */

/* ---------- Region: RP Request Flags (0x30-0x33, R->M) -------------------- */
#define REG_RP_REQ_FLAGS    (0x30u)
#define REG_RP_REQ_CODE     (0x31u)
#define REG_RP_REQ_ARG0     (0x32u)
#define REG_RP_REQ_ARG1     (0x33u)

/* ---------- Region: Non-RF Command Mailbox (0x34-0x5F, M->R) -------------- */
#define REG_CMD_MBOX_ID         (0x34u)
#define REG_CMD_MBOX_SEQ        (0x35u)
#define REG_CMD_MBOX_LEN        (0x36u)
#define REG_CMD_MBOX_FLAGS      (0x37u)
#define REG_CMD_MBOX_PAYLOAD    (0x38u)  /* 38 bytes: 0x38-0x5D */
#define REG_CMD_MBOX_CRC_L      (0x5Eu)
#define REG_CMD_MBOX_CRC_H      (0x5Fu)
#define CMD_MBOX_PAYLOAD_LEN    (38u)

/* ---------- Region: Response Mailbox (0x60-0x8F, R->M) -------------------- */
#define REG_RESP_ID         (0x60u)
#define REG_RESP_SEQ        (0x61u)
#define REG_RESP_STATUS     (0x62u)
#define REG_RESP_LEN        (0x63u)
#define REG_RESP_DATA       (0x64u)  /* 42 bytes: 0x64-0x8D */
#define REG_RESP_CRC_L      (0x8Eu)
#define REG_RESP_CRC_H      (0x8Fu)
#define RESP_DATA_LEN       (42u)

/* ---------- Region: Telemetry Snapshot (0x90-0xEF, R->M) ------------------ */
#define REG_TLM_UPTIME_S     (0x90u)  /* u32 RP uptime seconds */
#define REG_TLM_CONTACT_AGE  (0x94u)  /* u32 MSP contact age mirror */
#define REG_TLM_VBATT_MV     (0x98u)  /* u16 mV */
#define REG_TLM_IBATT_MA     (0x9Au)  /* i16 mA (signed) */
#define REG_TLM_PGEN_MW      (0x9Cu)  /* u16 mW */
#define REG_TLM_PLOAD_MW     (0x9Eu)  /* u16 mW */
#define REG_TLM_TPCB_CC      (0xA0u)  /* i16 centi-degC */
#define REG_TLM_TBATT_CC     (0xA2u)  /* i16 centi-degC */
#define REG_TLM_TX_SHUTDOWN  (0xA4u)  /* u8: 1 if final frame pending */
#define REG_TLM_MODE         (0xA5u)  /* u8: current RP mode */
#define REG_TLM_UPLINK_CNT   (0xA6u)  /* u16 valid uplink count */
#define REG_TLM_RSSI_X100    (0xA8u)  /* i16 RSSI in 0.01 dBm */
#define REG_TLM_SNR_X100     (0xAAu)  /* i16 SNR in 0.01 dB */
#define REG_TLM_FAULT_BITMAP (0xACu)  /* u16 RP fault bitmap summary */
#define REG_TLM_MSP_STATUS0  (0xAEu)
#define REG_TLM_MSP_STATUS1  (0xAFu)
/* 0xB0-0xED: reserved for future TM */
#define REG_TLM_CRC_L        (0xEEu)
#define REG_TLM_CRC_H        (0xEFu)

/* ---------- Region: Fault Bitmap / Counters (0xF0-0xFF, R->M) ------------- */
#define REG_FAULT_BITMAP_L      (0xF0u)
#define REG_FAULT_BITMAP_H      (0xF1u)
#define REG_FAULT_SEQ           (0xF2u)
#define REG_LATCH_FLAGS         (0xF3u)
#define REG_CNT_SPI_ERR         (0xF4u)
#define REG_CNT_I2C_ERR         (0xF5u)
#define REG_CNT_RADIO_TX_FAIL   (0xF6u)
#define REG_CNT_RADIO_RX_CRC    (0xF7u)
#define REG_CNT_MEM_CRC         (0xF8u)
#define REG_CNT_MEM_FULL        (0xF9u)
#define REG_CNT_CAN_ERR         (0xFAu)
#define REG_CNT_UART_OVERRUN    (0xFBu)
#define REG_CNT_CONTACT_ACK_TO  (0xFCu)
/* 0xFD: reserved */
#define REG_FAULT_CRC_L         (0xFEu)
#define REG_FAULT_CRC_H         (0xFFu)

/* ======================================================================
 * Enum / bitfield values (from xlsx image annotations)
 * ====================================================================== */

/* --- RP_STATE at 0x10 (R->M) ---
 * RP's actual current state reported to MSP. */
#define RP_STATE_UNINIT             (0x00u)
#define RP_STATE_BOOTING            (0x01u)
#define RP_STATE_INIT_HW            (0x02u)
#define RP_STATE_DEPLOY_WAIT_NO_TX  (0x03u)  /* useful for GDIR-MIS-03 */
#define RP_STATE_NOMINAL            (0x04u)
#define RP_STATE_SAFE               (0x05u)
#define RP_STATE_DEGRADED           (0x06u)
#define RP_STATE_MAINTENANCE        (0x07u)
#define RP_STATE_UPDATE_MODE        (0x08u)
#define RP_STATE_FAULT_HOLD         (0x09u)
#define RP_STATE_SHUTDOWN_PENDING   (0x0Au)  /* graceful quiesce before MSP cuts power */
#define RP_STATE_NOT_READY          (0xFEu)
#define RP_STATE_INVALID            (0xFFu)

/* --- LAST_ERROR at 0x17 (R->M) ---
 * RP "last thing that went wrong" summary for MSP diagnostics. */
#define RP_ERR_NONE                 (0x00u)
#define RP_ERR_I2C_LINK_TIMEOUT     (0x01u)
#define RP_ERR_MAILBOX_CRC_FAIL     (0x02u)
#define RP_ERR_RADIO_FAULT          (0x03u)
#define RP_ERR_STORAGE_FAULT        (0x04u)
#define RP_ERR_BUS_FAULT            (0x05u)
#define RP_ERR_ADCS_FAULT           (0x06u)
#define RP_ERR_ASSERT_OR_EXCEPTION  (0x07u)
#define RP_ERR_UPDATE_FAIL          (0x08u)
#define RP_ERR_CONFIG_REJECTED      (0x09u)
#define RP_ERR_CONTACT_ACK_TIMEOUT  (0x0Au)
#define RP_ERR_UNKNOWN              (0xFFu)

/* --- CONTACT_EVT_TYPE at 0x19 (R->M) ---
 * What kind of contact the RP thinks it saw.
 * For the 48h deadman, MSP should only accept VALID_TC_UPLINK
 * (and optionally VALID_FILE_TRANSFER_UPLINK). */
#define CONTACT_TYPE_NONE                   (0x00u)
#define CONTACT_TYPE_VALID_TC_UPLINK        (0x01u)  /* accept for deadman */
#define CONTACT_TYPE_VALID_NON_TC_UPLINK    (0x02u)
#define CONTACT_TYPE_VALID_FILE_XFER_UPLINK (0x03u)  /* optionally accept */
#define CONTACT_TYPE_GROUND_SESSION_START   (0x04u)
#define CONTACT_TYPE_GROUND_KEEPALIVE       (0x05u)
#define CONTACT_TYPE_UNKNOWN                (0xFFu)

/* CONTACT_EVT_FLAGS bits */
#define CONTACT_FLAG_AUTH_OK    (0x01u)  /* validated, authenticated uplink */
#define CONTACT_FLAG_CRC_OK     (0x02u)
#define CONTACT_FLAG_FRAME_OK   (0x04u)
#define CONTACT_FLAG_TC_CLASS   (0x08u)
/* bits [7:4]: spare */
/* Minimum flags required for deadman acceptance */
#define CONTACT_FLAGS_VALID_MIN (CONTACT_FLAG_AUTH_OK | CONTACT_FLAG_CRC_OK)

/* --- CONTACT_ACK_STATUS at 0x23 (M->R) ---
 * MSP writes this after processing the candidate contact event.
 * Granular rejection codes help debug why deadman didn't reset. */
#define CONTACT_ACK_NONE                (0x00u)
#define CONTACT_ACK_ACCEPTED            (0x01u)
#define CONTACT_ACK_REJECTED_BAD_FLAGS  (0x02u)
#define CONTACT_ACK_REJECTED_DUPLICATE  (0x03u)
#define CONTACT_ACK_REJECTED_STALE      (0x04u)
#define CONTACT_ACK_REJECTED_POLICY     (0x05u)
#define CONTACT_ACK_REJECTED_INVTYPE    (0x06u)
#define CONTACT_ACK_ERROR               (0xFFu)

/* --- MSP_STATUS0 at 0x20 (M->R) ---
 * Packed supervisor status byte 0. */
#define MSP_STATUS0_MODE_MASK       (0x07u)  /* bits [2:0] = current MSP mode */
#define MSP_STATUS0_WDT_OK          (0x08u)  /* external WDTs healthy */
#define MSP_STATUS0_BATT_OK         (0x10u)  /* battery voltage OK */
#define MSP_STATUS0_RAIL_3V3_OK     (0x20u)  /* 3V3 rail PG */
#define MSP_STATUS0_FAULT           (0x80u)  /* any active fault */

/* MSP mode codes — defined as msp_mode_t enum in mode_state_machine.h */

/* --- MODE_CMD at 0x28 (M->R) ---
 * MSP-requested operating mode for RP. */
#define MODE_CMD_NO_OP                  (0x00u)
#define MODE_CMD_ENTER_STARTUP_MINIMUM  (0x01u)
#define MODE_CMD_ENTER_NOMINAL          (0x02u)
#define MODE_CMD_ENTER_SAFE             (0x03u)
#define MODE_CMD_PREPARE_FOR_SURVIVAL   (0x04u)  /* RP stops, then MSP powers it off */
#define MODE_CMD_ENTER_MAINTENANCE      (0x05u)
#define MODE_CMD_QUIESCE_NONESSENTIAL   (0x06u)  /* "not full SAFE but reduce activity" */
#define MODE_CMD_RELOAD_CONFIG          (0x07u)
#define MODE_CMD_INVALID                (0xFFu)

/* --- POWER_CMD at 0x29 (M->R) ---
 * Works together with RAIL_MASK_LO/HI. */
#define POWER_CMD_NO_OP                 (0x00u)
#define POWER_CMD_APPLY_RAIL_MASK       (0x01u)
#define POWER_CMD_ENABLE_MASKED_RAILS   (0x02u)
#define POWER_CMD_DISABLE_MASKED_RAILS  (0x03u)
#define POWER_CMD_CYCLE_MASKED_RAILS    (0x04u)
#define POWER_CMD_LATCH_OFF_MASKED      (0x05u)
#define POWER_CMD_CLEAR_LATCH_OFF       (0x06u)
#define POWER_CMD_REFRESH_STATUS        (0x07u)
#define POWER_CMD_INVALID               (0xFFu)

/* --- RESET_CMD at 0x2A (M->R) ---
 * Non-power-cut resets / local recovery on RP.
 * Full hard reset / power-cycle done by MSP via ~RESET_RP / EN_3V3. */
#define RESET_CMD_NO_OP                     (0x00u)
#define RESET_CMD_RESET_PERIPHERALS_SOFT    (0x01u)
#define RESET_CMD_RESET_BUS_INTERFACES      (0x02u)
#define RESET_CMD_SELF_SOFT_RESET           (0x03u)  /* RP resets internally */
#define RESET_CMD_CLEAR_FAULT_LATCHES       (0x04u)
#define RESET_CMD_CLEAR_NONCRITICAL_CTRS    (0x05u)
#define RESET_CMD_CLEAR_MAILBOXES           (0x06u)
#define RESET_CMD_REINIT_DRIVERS            (0x07u)
#define RESET_CMD_INVALID                   (0xFFu)

/* --- BOOT_CMD at 0x2B (M->R) ---
 * A/B image behaviour or golden image policy. */
#define BOOT_CMD_NO_OP                  (0x00u)
#define BOOT_CMD_BOOT_CURRENT_SLOT      (0x01u)
#define BOOT_CMD_MARK_SLOT_A_NEXT       (0x02u)
#define BOOT_CMD_MARK_SLOT_B_NEXT       (0x03u)
#define BOOT_CMD_BOOT_GOLDEN_NEXT       (0x04u)
#define BOOT_CMD_CONFIRM_CURRENT_IMAGE  (0x05u)
#define BOOT_CMD_ROLLBACK_TO_PREVIOUS   (0x06u)
#define BOOT_CMD_QUERY_BOOT_STATUS      (0x07u)
#define BOOT_CMD_INVALID                (0xFFu)
/* Simplified values if not using A/B images: */
#define BOOT_CMD_S_BOOT_CURRENT         (0x01u)
#define BOOT_CMD_S_BOOT_GOLDEN_NEXT     (0x02u)
#define BOOT_CMD_S_CONFIRM_CURRENT      (0x03u)
#define BOOT_CMD_S_ROLLBACK_GOLDEN      (0x04u)

/* --- CMD_STATUS at 0x2F (R->M) ---
 * BITFIELD (not an enum) — RP can communicate multiple conditions at once. */
#define CMD_STATUS_ACK_VALID        (0x01u)  /* bit0: command acknowledged */
#define CMD_STATUS_APPLIED          (0x02u)  /* bit1: command applied */
#define CMD_STATUS_REJECTED         (0x04u)  /* bit2: command rejected */
#define CMD_STATUS_BUSY             (0x08u)  /* bit3: RP busy, retry later */
#define CMD_STATUS_FAULT_PRESENT    (0x10u)  /* bit4: fault active on RP */
#define CMD_STATUS_REQUIRES_RESET   (0x20u)  /* bit5: action needs reset */
/* bits [7:6]: reserved */

/* --- RP_REQ_FLAGS at 0x30 (R->M) ---
 * Sticky bits set by RP, cleared by MSP after ack. */
#define RP_REQ_PENDING          (0x01u)  /* bit0: request is pending */
#define RP_REQ_URGENT           (0x02u)  /* bit1: urgent, handle ASAP */
#define RP_REQ_CLEAR_AFTER_ACK  (0x04u)  /* bit2: auto-clear on ack */
/* bits [7:3]: spare */

/* --- RP_REQ_CODE at 0x31 (R->M) ---
 * RP asking MSP to do something only MSP can do. */
#define RP_REQ_NONE                     (0x00u)
#define RP_REQ_POWER_CYCLE_RP           (0x01u)
#define RP_REQ_CLEAR_FAULT_COUNTERS     (0x02u)
#define RP_REQ_HEATER_POLICY_REFRESH    (0x03u)
#define RP_REQ_SAFE_MODE_CONFIRM        (0x04u)
#define RP_REQ_CONFIG_COMMIT            (0x05u)
#define RP_REQ_TIME_SYNC                (0x06u)
#define RP_REQ_EVENT_LOG_SNAPSHOT       (0x07u)
#define RP_REQ_BOOT_STATUS_REFRESH      (0x08u)
#define RP_REQ_UNKNOWN                  (0xFFu)

/* --- RESP_STATUS at 0x62 (R->M) --- */
#define RESP_STATUS_OK              (0x00u)
#define RESP_STATUS_BUSY            (0x01u)
#define RESP_STATUS_BAD_CMD         (0x02u)
#define RESP_STATUS_BAD_LEN         (0x03u)
#define RESP_STATUS_BAD_CRC         (0x04u)
#define RESP_STATUS_BAD_STATE       (0x05u)
#define RESP_STATUS_NOT_ALLOWED     (0x06u)
#define RESP_STATUS_NOT_FOUND       (0x07u)
#define RESP_STATUS_HW_FAIL         (0x08u)
#define RESP_STATUS_TIMEOUT         (0x09u)
#define RESP_STATUS_PARTIAL         (0x0Au)
#define RESP_STATUS_RETRY           (0x0Bu)
#define RESP_STATUS_INTERNAL_ERROR  (0xFFu)

/* --- RAIL_MASK_LO bits [7:0] ---
 * Used with POWER_CMD to select which rails to act on. */
#define RAIL_5V_PAYLOAD_EN      (0x01u)  /* bit0 */
#define RAIL_12V_PAYLOAD_EN     (0x02u)  /* bit1 */
#define RAIL_UHF_AUX_EN         (0x04u)  /* bit2 */
#define RAIL_SENSOR_BUS_EN      (0x08u)  /* bit3 */
#define RAIL_HEATER_AUX_EN      (0x10u)  /* bit4 */
#define RAIL_STORAGE_AUX_EN     (0x20u)  /* bit5 */
#define RAIL_EXPANDER_DOMAIN_EN (0x40u)  /* bit6 */
/* bit7: reserved */
/* RAIL_MASK_HI bits [7:0] map to rails [15:8] — TBD per power tree */

/* --- Fault bitmap bits (FAULT_BITMAP_RP at 0xF0-0xF1, R->M) ---
 * 16-bit bitmap of RP-owned active faults.
 * Bit definitions match the "Fault Counters" sheet in the task schedule.
 * Each bit set = fault currently active. Use LATCH_FLAGS (0xF3) for
 * "ever seen" summary that does not auto-clear.
 * See fault_counters.h for full fault metadata (severity, decay, owner). */
#define RP_FAULT_SPI_ERR            (0x0001u)  /* bit0  F_SPI_ERR        P2 */
#define RP_FAULT_I2C_LINK_ERR       (0x0002u)  /* bit1  F_I2C_LINK_ERR   P1 (shared) */
#define RP_FAULT_RADIO_TX_FAIL      (0x0004u)  /* bit2  F_RADIO_TX_FAIL  P2 */
#define RP_FAULT_RADIO_RX_CRC       (0x0008u)  /* bit3  F_RADIO_RX_CRC   P3 */
#define RP_FAULT_TX_SHUTDOWN_IMM    (0x0010u)  /* bit4  F_TX_SHUTDOWN_IMM  P0 latching */
#define RP_FAULT_TX_SHUTDOWN_PERM   (0x0020u)  /* bit5  F_TX_SHUTDOWN_PERM P0 latching */
#define RP_FAULT_DEPLOY_NO_TX       (0x0040u)  /* bit6  F_DEPLOY_NO_TX_ACTIVE P1 state */
#define RP_FAULT_MEM_CRC            (0x0080u)  /* bit7  F_MEM_CRC        P1 */
#define RP_FAULT_MEM_FULL           (0x0100u)  /* bit8  F_MEM_FULL       P3 latching */
#define RP_FAULT_CAN0_ERR           (0x0200u)  /* bit9  F_CAN0_ERR       P2 */
#define RP_FAULT_CAN1_ERR           (0x0400u)  /* bit10 F_CAN1_ERR       P2 */
#define RP_FAULT_UART_OVERRUN       (0x0800u)  /* bit11 F_UART_OVERRUN   P3 */
#define RP_FAULT_ADCS_FAULT         (0x1000u)  /* bit12 F_ADCS_FAULT     P2 */
#define RP_FAULT_CONTACT_ACK_TO     (0x2000u)  /* bit13 F_CONTACT_ACK_TIMEOUT P2 */
#define RP_FAULT_CONTACT_REJECTED   (0x4000u)  /* bit14 F_CONTACT_REJECTED   P3 */
#define RP_FAULT_I2C_ISOLATION      (0x8000u)  /* bit15 F_I2C_ISOLATION_EVENT */

/* Mask of P0 RP faults that MSP should treat as escalation triggers */
#define RP_FAULT_P0_MASK  (RP_FAULT_TX_SHUTDOWN_IMM | RP_FAULT_TX_SHUTDOWN_PERM)

#endif /* RP_REGMAP_H_ */
