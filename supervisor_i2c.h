/*
 * supervisor_i2c.h
 *
 * Polling I2C master driver for MSP430FR UCB0.
 * Used by the supervisor to read/write the RP2350 register map
 * and to access battery sensors (INA219, MCP9808) on the same bus.
 *
 * Bus: I2C1 (shared), MSP is master.
 * Pins: P1.6 = UCB0SDA, P1.7 = UCB0SCL (configured in Init_GPIO).
 * Clock: SMCLK 8 MHz, 400 kHz I2C.
 *
 * All functions are blocking/polling with a hardware timeout (~31 ms).
 * Call only from task context, not from ISR.
 */

#ifndef SUPERVISOR_I2C_H_
#define SUPERVISOR_I2C_H_

#include <stdint.h>

/* Return codes */
#define I2C_OK          (0)
#define I2C_ERR_NACK    (-1)  /* slave NACKed the address or data */
#define I2C_ERR_TIMEOUT (-2)  /* CLK low-timeout (bus stuck / no slave) */
#define I2C_ERR_BUSY    (-3)  /* bus busy at start of transaction */
#define I2C_ERR_ARG     (-4)  /* invalid argument (e.g. len == 0) */

/*
 * Initialise UCB0 as I2C master at 400 kHz.
 * Must be called after Init_Clock() and after GPIO pins are already
 * configured as peripheral (done in Init_GPIO).
 */
void supervisor_i2c_init(void);

/*
 * Write `len` bytes from `data` to slave `addr` starting at register `reg`.
 * Generates: START | addr+W | reg | data[0] .. data[len-1] | STOP
 * Returns I2C_OK or a negative error code.
 */
int8_t i2c_write_reg(uint8_t addr, uint8_t reg, const uint8_t *data, uint8_t len);

/*
 * Read `len` bytes from slave `addr` starting at register `reg` into `buf`.
 * Generates: START | addr+W | reg | RSTART | addr+R | data[0..len-1] | STOP
 * Returns I2C_OK or a negative error code.
 */
int8_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);

/*
 * Convenience: write a single byte to one register.
 */
static inline int8_t i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t val)
{
    return i2c_write_reg(addr, reg, &val, 1u);
}

/*
 * Convenience: read a single byte from one register.
 * Returns the byte value (0-255) on success, or a negative error code.
 */
static inline int16_t i2c_read_byte(uint8_t addr, uint8_t reg)
{
    uint8_t val;
    int8_t  rc = i2c_read_reg(addr, reg, &val, 1u);
    return (rc == I2C_OK) ? (int16_t)val : (int16_t)rc;
}

/*
 * Attempt bus recovery: 9 manual SCL pulses to unstick a held-low SDA.
 * Call if i2c_read_reg / i2c_write_reg repeatedly returns I2C_ERR_BUSY.
 */
void i2c_bus_recover(void);

/* ------------------------------------------------------------------
 * Level-1 retry wrappers (FDIR document §"Level 1: retry up to 3 times")
 *
 * Re-attempt the transaction up to I2C_RETRY_COUNT times on any error.
 * The hardware 31 ms clock-low timeout provides natural pacing: after a
 * timeout the EUSCI resets before the next attempt is issued.
 * Use for critical reads/writes where a single transient NACK or glitch
 * should not immediately trigger fault escalation.
 * ------------------------------------------------------------------ */
#define I2C_RETRY_COUNT     (3u)

static inline int8_t i2c_write_reg_retry(uint8_t addr, uint8_t reg,
                                          const uint8_t *data, uint8_t len)
{
    uint8_t i;
    int8_t  rc = I2C_ERR_ARG;
    for (i = 0u; i < I2C_RETRY_COUNT; i++) {
        rc = i2c_write_reg(addr, reg, data, len);
        if (rc == I2C_OK) { break; }
    }
    return rc;
}

static inline int8_t i2c_read_reg_retry(uint8_t addr, uint8_t reg,
                                          uint8_t *buf, uint8_t len)
{
    uint8_t i;
    int8_t  rc = I2C_ERR_ARG;
    for (i = 0u; i < I2C_RETRY_COUNT; i++) {
        rc = i2c_read_reg(addr, reg, buf, len);
        if (rc == I2C_OK) { break; }
    }
    return rc;
}

#endif /* SUPERVISOR_I2C_H_ */
