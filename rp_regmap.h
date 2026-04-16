/*
 * rp_regmap.h
 *
 * RP2350 I2C register map — addresses and field definitions.
 * Layout matches MSP_RP_I2C_Register_Map.csv (full 256-byte spec).
 *
 * Ownership convention:
 *   RP_OWNED  : RP writes locally; MSP reads via i2c_read_reg()
 *   MSP_OWNED : MSP writes via i2c_write_reg(); RP stores in its _mem[]
 */

#ifndef RP_REGMAP_H_
#define RP_REGMAP_H_

/* RP2350 I2C slave address */
#define RP_I2C_ADDR             (0x42u)

/* ------------------------------------------------------------------
 * Header  0x00–0x0F  (RP_OWNED — RP writes at boot, MSP reads to verify)
 * ------------------------------------------------------------------ */
#define REG_MAGIC0              (0x00u)   /* 'H' = 0x48                      */
#define REG_MAGIC1              (0x01u)   /* 'I' = 0x49                      */
#define REG_MAGIC2              (0x02u)   /* 'A' = 0x41                      */
#define REG_MAGIC3              (0x03u)   /* '1' = 0x31                      */
#define REG_REGMAP_VERSION      (0x04u)   /* format version                  */
#define REG_REGMAP_LEN_L        (0x05u)   /* total length LSB (0x00 = 256)   */
#define REG_REGMAP_LEN_H        (0x06u)   /* total length MSB (0x01 = 256)   */
#define REG_ENDIAN              (0x07u)   /* 0 = little-endian               */
#define REG_WDT_PERIOD_S        (0x08u)   /* RP heartbeat period in seconds  */
#define REG_I2C_ADDR_HDR        (0x0Cu)   /* self-reported I2C address       */
#define REG_HDR_CRC_L           (0x0Eu)   /* CRC16 over 0x00–0x0D LSB        */
#define REG_HDR_CRC_H           (0x0Fu)   /* CRC16 over 0x00–0x0D MSB        */

#define HDR_MAGIC0_VAL          (0x48u)   /* 'H' */
#define HDR_MAGIC1_VAL          (0x49u)   /* 'I' */
#define HDR_MAGIC2_VAL          (0x41u)   /* 'A' */
#define HDR_MAGIC3_VAL          (0x31u)   /* '1' */
#define HDR_REGMAP_VERSION_VAL  (0x01u)

/* ------------------------------------------------------------------
 * RP State / Health  0x10–0x17  (RP_OWNED)
 * ------------------------------------------------------------------ */
#define REG_RP_STATE            (0x10u)   /* RP state enum (see below)       */
#define REG_RP_UPTIME0          (0x11u)   /* uptime seconds byte 0 (LSB)     */
#define REG_RP_UPTIME1          (0x12u)   /* uptime seconds byte 1           */
#define REG_RP_UPTIME2          (0x13u)   /* uptime seconds byte 2           */
#define REG_RP_UPTIME3          (0x14u)   /* uptime seconds byte 3 (MSB)     */
#define REG_RP_HB0              (0x15u)   /* heartbeat counter LSB           */
#define REG_RP_HB1              (0x16u)   /* heartbeat counter MSB           */
#define REG_RP_LAST_ERROR       (0x17u)   /* last RP error code              */

/* RP_STATE enum values */
#define RP_STATE_BOOT           (0x00u)
#define RP_STATE_INIT           (0x01u)
#define RP_STATE_NOMINAL        (0x02u)
#define RP_STATE_SAFE           (0x03u)
#define RP_STATE_FAULT          (0x04u)

/* ------------------------------------------------------------------
 * Ground Contact Candidate  0x18–0x1F  (RP_OWNED — stub on dev board)
 * ------------------------------------------------------------------ */
#define REG_CONTACT_EVT_SEQ     (0x18u)   /* monotonic event sequence        */
#define REG_CONTACT_EVT_TYPE    (0x19u)   /* event type enum                 */
#define REG_CONTACT_EVT_FLAGS   (0x1Au)   /* auth/CRC/frame flags            */
#define REG_CONTACT_EVT_PENDING (0x1Bu)   /* 1 = pending, 0 = none           */
#define REG_CONTACT_EVT_TIME0   (0x1Cu)   /* event time byte 0               */
#define REG_CONTACT_EVT_TIME1   (0x1Du)   /* event time byte 1               */
#define REG_CONTACT_EVT_TIME2   (0x1Eu)   /* event time byte 2               */
#define REG_CONTACT_EVT_TIME3   (0x1Fu)   /* event time byte 3               */

/* ------------------------------------------------------------------
 * MSP Status  0x20–0x21  (MSP_OWNED — MSP writes)
 * ------------------------------------------------------------------ */
#define REG_MSP_STATUS0         (0x20u)   /* mode[2:0] | WDT_OK | BATT_OK | RAIL_OK */
#define REG_MSP_STATUS1         (0x21u)   /* extended fault flags            */

/* ------------------------------------------------------------------
 * Ground Contact Ack / Timer  0x22–0x27  (MSP_OWNED — MSP writes)
 * ------------------------------------------------------------------ */
#define REG_CONTACT_ACK_SEQ     (0x22u)   /* last EVT_SEQ processed by MSP   */
#define REG_CONTACT_ACK_STATUS  (0x23u)   /* 0=None 1=Accepted 2=Rejected    */
#define REG_LAST_CONTACT_AGE0   (0x24u)   /* age since last contact byte 0   */
#define REG_LAST_CONTACT_AGE1   (0x25u)   /* age byte 1                      */
#define REG_LAST_CONTACT_AGE2   (0x26u)   /* age byte 2                      */
#define REG_LAST_CONTACT_AGE3   (0x27u)   /* age byte 3                      */

/* ------------------------------------------------------------------
 * Mode / Power Commands  0x28–0x2F  (MSP_OWNED except CMD_STATUS)
 * ------------------------------------------------------------------ */
#define REG_MODE_CMD            (0x28u)   /* requested mode enum             */
#define REG_POWER_CMD           (0x29u)   /* power action                    */
#define REG_RESET_CMD           (0x2Au)   /* reset request                   */
#define REG_BOOT_CMD            (0x2Bu)   /* boot image/behavior             */
#define REG_RAIL_MASK_LO        (0x2Cu)   /* rail allow mask [7:0]           */
#define REG_RAIL_MASK_HI        (0x2Du)   /* rail allow mask [15:8]          */
#define REG_CMD_SEQ             (0x2Eu)   /* monotonic command sequence      */
#define REG_CMD_STATUS          (0x2Fu)   /* RP_OWNED: RP ack for CMD_SEQ    */

/* CMD_STATUS values */
#define CMD_STATUS_NONE         (0x00u)
#define CMD_STATUS_ACK          (0x01u)
#define CMD_STATUS_BUSY         (0x02u)
#define CMD_STATUS_ERR          (0x03u)

/* ------------------------------------------------------------------
 * RP Request Flags  0x30–0x33  (RP_OWNED — RP writes, MSP reads)
 * ------------------------------------------------------------------ */
#define REG_RP_REQ_FLAGS        (0x30u)   /* request bitmask (sticky)        */
#define REG_RP_REQ_CODE         (0x31u)   /* request code enum               */
#define REG_RP_REQ_ARG0         (0x32u)   /* argument 0                      */
#define REG_RP_REQ_ARG1         (0x33u)   /* argument 1                      */

/* REQ_FLAGS bit definitions */
#define RP_REQ_PENDING          (1u << 0)
#define RP_REQ_URGENT           (1u << 1)
#define RP_REQ_CLEAR_AFTER_ACK  (1u << 2)

/* ------------------------------------------------------------------
 * Telemetry Snapshot  0x90–0xAF  (MSP_OWNED — MSP writes)
 * ------------------------------------------------------------------ */
#define REG_TLM_VBATT_MV        (0x98u)   /* uint16 LE, millivolts           */
#define REG_TLM_IBATT_MA        (0x9Au)   /* int16  LE, milliamps            */
#define REG_TLM_TBATT_CC        (0xA2u)   /* int16  LE, centi-°C (stub=0)   */
#define REG_TLM_MODE            (0xA5u)   /* MSP operating mode enum         */
#define REG_TLM_MSP_STATUS0     (0xAEu)   /* mirror of REG_MSP_STATUS0       */
#define REG_TLM_MSP_STATUS1     (0xAFu)   /* mirror of REG_MSP_STATUS1       */

/* ------------------------------------------------------------------
 * Fault Bitmap / Counters  0xF0–0xFF  (RP_OWNED — RP writes, MSP reads)
 * ------------------------------------------------------------------ */
#define REG_FAULT_BITMAP_L      (0xF0u)   /* RP fault bitmap LSB             */
#define REG_FAULT_BITMAP_H      (0xF1u)   /* RP fault bitmap MSB             */
#define REG_FAULT_SEQ           (0xF2u)   /* monotonic, increments on change */
#define REG_LATCH_FLAGS         (0xF3u)   /* latched / ever-seen faults      */
#define REG_CNT_I2C_ERR         (0xF5u)   /* I2C link error counter          */
#define REG_CNT_CONTACT_TIMEOUT (0xFCu)   /* contact ack timeout counter     */
#define REG_FAULT_CRC_L         (0xFEu)   /* CRC16 over 0xF0–0xFD LSB        */
#define REG_FAULT_CRC_H         (0xFFu)   /* CRC16 over 0xF0–0xFD MSB        */

/* ------------------------------------------------------------------
 * STATUS0 bit fields  (shared between MSP_STATUS0 and RP status byte)
 * ------------------------------------------------------------------ */
#define STATUS0_MODE_SHIFT      (0u)
#define STATUS0_MODE_MASK       (0x07u)
#define STATUS0_WDT_OK          (1u << 3)
#define STATUS0_BATT_OK         (1u << 4)
#define STATUS0_RAIL_OK         (1u << 5)

/* ------------------------------------------------------------------
 * STATUS1 bit fields
 * ------------------------------------------------------------------ */
#define STATUS1_OC_MCU          (1u << 0)   /* eFuse OC on 3V3 MSP rail      */
#define STATUS1_UV_LOAD         (1u << 1)   /* regulator PG lost             */

/* ------------------------------------------------------------------
 * MSP fault bitmap bit fields (MSP-local; reflected in STATUS1 and
 * stored in g_fault / g_rp_reg_snapshot for JTAG inspection)
 * ------------------------------------------------------------------ */
#define FAULT_BIT_WDT_RP_MISS   (1u << 0)   /* RP reset counter tripped       */
#define FAULT_BIT_WDT_EXT_TRIP  (1u << 1)   /* RST-pin reset detected at boot  */
#define FAULT_BIT_PWR_OC_MCU    (1u << 2)   /* 3V3_MSP overcurrent (>100 mA)  */
#define FAULT_BIT_PWR_UV_LOAD   (1u << 3)   /* bus voltage undervoltage        */

/* ------------------------------------------------------------------
 * MSP mode enum (written to REG_TLM_MODE and mode field of STATUS0)
 * ------------------------------------------------------------------ */
#define MODE_STARTUP            (0x00u)
#define MODE_SAFE               (0x01u)
#define MODE_NOMINAL            (0x02u)
#define MODE_COMMS              (0x03u)
#define MODE_LOW_POWER          (0x04u)
#define MODE_FAULT              (0x05u)

#endif /* RP_REGMAP_H_ */
