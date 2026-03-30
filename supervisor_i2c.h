/*
 * supervisor_i2c.h
 *
 * Polling I2C master driver for MSP430FR5969 UCB0.
 * Bus: P1.6 = SDA, P1.7 = SCL (configured as peripheral in Init_GPIO).
 * Clock: SMCLK 8 MHz -> 400 kHz I2C.
 * Hardware clock-low timeout ~31 ms prevents bus hang.
 *
 * All functions are blocking. Call only from task context, not from ISR.
 */

#ifndef SUPERVISOR_I2C_H_
#define SUPERVISOR_I2C_H_

#include <stdint.h>

/* Return codes */
#define I2C_OK           ( 0)
#define I2C_ERR_NACK     (-1)
#define I2C_ERR_TIMEOUT  (-2)
#define I2C_ERR_BUSY     (-3)
#define I2C_ERR_ARG      (-4)

void   supervisor_i2c_init(void);
int8_t i2c_write_reg(uint8_t addr, uint8_t reg, const uint8_t *data, uint8_t len);
int8_t i2c_read_reg (uint8_t addr, uint8_t reg, uint8_t *buf,        uint8_t len);

#endif /* SUPERVISOR_I2C_H_ */
