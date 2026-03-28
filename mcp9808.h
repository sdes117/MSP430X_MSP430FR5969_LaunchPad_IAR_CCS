/*
 * mcp9808.h
 *
 * Minimal I2C driver for the Microchip MCP9808 digital temperature sensor.
 * Used for battery temperature sensing on the BATT_SENSE I2C bus.
 *
 * Resolution: 0.0625 °C per LSB (13-bit signed value).
 * Outputs temperature in centi-degrees Celsius (°C * 100) for integer math.
 */

#ifndef MCP9808_H_
#define MCP9808_H_

#include <stdint.h>

/* ------------------------------------------------------------------
 * Default I2C address (A2=A1=A0=GND → 0x18).
 * ------------------------------------------------------------------ */
#ifndef MCP9808_ADDR
#define MCP9808_ADDR            (0x18u)
#endif

/* ------------------------------------------------------------------
 * Register addresses (from MCP9808 datasheet Table 5-1)
 * ------------------------------------------------------------------ */
#define MCP9808_REG_CONFIG      (0x01u)
#define MCP9808_REG_UPPER_TEMP  (0x02u)
#define MCP9808_REG_LOWER_TEMP  (0x03u)
#define MCP9808_REG_CRIT_TEMP   (0x04u)
#define MCP9808_REG_TEMP        (0x05u)  /* ambient temperature — read this */
#define MCP9808_REG_MANUF_ID    (0x06u)  /* should read 0x0054 */
#define MCP9808_REG_DEVICE_ID   (0x07u)  /* should read 0x0400 */

/* Expected manufacturer ID for validation */
#define MCP9808_MANUF_ID        (0x0054u)
/* Expected device ID (upper byte = 0x04) */
#define MCP9808_DEVICE_ID_MASK  (0xFF00u)
#define MCP9808_DEVICE_ID_VAL   (0x0400u)

/* ------------------------------------------------------------------
 * Return codes
 * ------------------------------------------------------------------ */
#define MCP9808_OK              (0)
#define MCP9808_ERR             (-1)

/* ------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------ */

/*
 * Validate device presence by reading manufacturer + device ID registers.
 * Returns MCP9808_OK if IDs match, MCP9808_ERR otherwise.
 * Call once at startup (msp_self_test or battery_monitor init).
 */
int8_t mcp9808_check_id(void);

/*
 * Read ambient temperature.
 *
 * temp_cc_out: temperature in centi-degC (e.g. 2500 = 25.00 °C, -1000 = -10.00 °C)
 *
 * Returns MCP9808_OK on success, MCP9808_ERR on I2C failure.
 */
int8_t mcp9808_read_temp(int16_t *temp_cc_out);

#endif /* MCP9808_H_ */
