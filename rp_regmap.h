/*
 * rp_regmap.h
 *
 * RP2350 I2C register map — addresses and field definitions.
 * The MSP writes to these as the I2C master; RP is the slave at RP_I2C_ADDR.
 *
 * Writes will NACK until the RP is powered and programmed, but the MSP
 * can attempt them unconditionally. When the RP comes online it will
 * immediately start seeing the correct values.
 */

#ifndef RP_REGMAP_H_
#define RP_REGMAP_H_

/* RP2350 I2C slave address */
#define RP_I2C_ADDR             (0x42u)

/* Ownership convention:
 *   MSP_OWNED : MSP writes via i2c_write_reg, RP stores locally
 *   RP_OWNED  : RP writes locally, MSP reads via i2c_read_reg on request
 */

/* ------------------------------------------------------------------
 * RP_OWNED  0x40–0x5F : RP status and health (MSP reads these)
 * ------------------------------------------------------------------ */
#define REG_RP_STATUS0          (0x40u)   /* mode[2:0] | tasks_ok | wdt_ok */
#define REG_RP_STATUS1          (0x41u)   /* RP fault flags                */
#define REG_RP_UPTIME_LO        (0x44u)   /* uint16 LE uptime seconds [15:0]  */
#define REG_RP_UPTIME_HI        (0x46u)   /* uint16 LE uptime seconds [31:16] */
#define REG_RP_WDT_CTR          (0x48u)   /* uint8 WDT kick counter            */
#define REG_RP_CONTACT_FLAG     (0x50u)   /* uint8 bit0 = contact pending      */

/* RP_STATUS0 bit fields */
#define RP_STATUS0_TASKS_OK     (1u << 3)
#define RP_STATUS0_WDT_OK       (1u << 4)

/* ------------------------------------------------------------------
 * MSP_OWNED : MSP -> RP status registers (MSP writes, RP reads)
 * ------------------------------------------------------------------ */

/* MSP operating mode + fault summary */
#define REG_MSP_STATUS0         (0x20u)   /* mode[2:0], fault flags[4:0] */
#define REG_MSP_STATUS1         (0x21u)   /* extended fault flags */

/* Mode/power command block (MSP writes to instruct RP) */
#define REG_MODE_CMD            (0x28u)
#define REG_POWER_CMD           (0x29u)
#define REG_RESET_CMD           (0x2Au)
#define REG_BOOT_CMD            (0x2Bu)
#define REG_RAIL_MASK_LO        (0x2Cu)
#define REG_RAIL_MASK_HI        (0x2Du)
#define REG_CMD_SEQ             (0x2Eu)

/* ------------------------------------------------------------------
 * Telemetry snapshot (MSP writes sensor readings here for RP downlink)
 * ------------------------------------------------------------------ */
#define REG_TLM_VBATT_MV        (0x98u)   /* uint16 LE, millivolts */
#define REG_TLM_IBATT_MA        (0x9Au)   /* int16  LE, milliamps  */
#define REG_TLM_TBATT_CC        (0xA2u)   /* int16  LE, centi-°C   */
#define REG_TLM_MODE            (0xA5u)   /* MSP operating mode    */
#define REG_TLM_MSP_STATUS0     (0xAEu)   /* mirror of REG_MSP_STATUS0 */
#define REG_TLM_MSP_STATUS1     (0xAFu)   /* mirror of REG_MSP_STATUS1 */

/* ------------------------------------------------------------------
 * STATUS0 bit fields
 * ------------------------------------------------------------------ */
#define STATUS0_MODE_SHIFT      (0u)
#define STATUS0_MODE_MASK       (0x07u)
#define STATUS0_WDT_OK          (1u << 3)
#define STATUS0_BATT_OK         (1u << 4)
#define STATUS0_RAIL_OK         (1u << 5)

/* ------------------------------------------------------------------
 * STATUS1 bit fields
 * ------------------------------------------------------------------ */
#define STATUS1_OC_MCU          (1u << 0)   /* eFuse OC on 3V3 MSP rail */
#define STATUS1_UV_LOAD         (1u << 1)   /* regulator PG lost */

#endif /* RP_REGMAP_H_ */
