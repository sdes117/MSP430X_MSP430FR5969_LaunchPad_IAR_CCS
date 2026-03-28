/*
 * ina219.h
 *
 * Minimal I2C driver for the Texas Instruments INA219 current/power monitor.
 * Used for battery voltage and current sensing on the BATT_SENSE I2C bus.
 *
 * Assumes the supervisor_i2c driver (UCB0 master) is already initialised.
 * All register accesses are big-endian (MSB first), unlike most MSP430 peripherals.
 */

#ifndef INA219_H_
#define INA219_H_

#include <stdint.h>

/* ------------------------------------------------------------------
 * Default I2C address (A0=GND, A1=GND → 0x40).
 * Override with INA219_ADDR define before including if different.
 * ------------------------------------------------------------------ */
#ifndef INA219_ADDR
#define INA219_ADDR             (0x40u)
#endif

/* ------------------------------------------------------------------
 * Register addresses
 * ------------------------------------------------------------------ */
#define INA219_REG_CONFIG       (0x00u)
#define INA219_REG_SHUNT_V      (0x01u)  /* signed, 10 uV/LSB */
#define INA219_REG_BUS_V        (0x02u)  /* bits[15:3] * 4mV; bit1=CNVR; bit0=OVF */
#define INA219_REG_POWER        (0x03u)  /* unsigned, Power_LSB = 20 * Current_LSB */
#define INA219_REG_CURRENT      (0x04u)  /* signed, Current_LSB configurable */
#define INA219_REG_CALIB        (0x05u)

/* ------------------------------------------------------------------
 * Configuration register value
 *   Bits [15:13] = 000  (reset = 0)
 *   Bit  [13]    = 0    (not reset)
 *   Bits [12:11] = 11   (bus range = 32V)
 *   Bits [10:9]  = 11   (gain = /8, shunt range ±320mV)
 *   Bits [8:7]   = 11   (bus ADC = 12-bit, 532us)
 *   Bits [6:3]   = 1111 (shunt ADC = 12-bit, 532us)
 *   Bits [2:0]   = 111  (continuous shunt + bus)
 * Value: 0x3FFF
 * ------------------------------------------------------------------ */
#define INA219_CONFIG_DEFAULT   (0x3FFFu)

/* ------------------------------------------------------------------
 * Calibration constant
 *
 * CAL = trunc(0.04096 / (Current_LSB * Rshunt))
 *
 * Default tuning: Rshunt = 0.100 Ω, max current = 2.0 A
 *   Current_LSB = 2.0 A / 32768 ≈ 61.04 uA/LSB
 *   CAL = trunc(0.04096 / (61.04e-6 * 0.100)) = 6710
 *
 * Override INA219_CALIB_VALUE and INA219_CURRENT_LSB_UA to match your shunt.
 * ------------------------------------------------------------------ */
#ifndef INA219_CALIB_VALUE
#define INA219_CALIB_VALUE      (6710u)
#endif

/* Current LSB in microamps — must match CAL above */
#ifndef INA219_CURRENT_LSB_UA
#define INA219_CURRENT_LSB_UA   (61u)   /* 61 uA/LSB */
#endif

/* Bus voltage LSB is always 4 mV (fixed by INA219 hardware). */
#define INA219_BUS_V_LSB_MV     (4u)

/* ------------------------------------------------------------------
 * Return codes
 * ------------------------------------------------------------------ */
#define INA219_OK               (0)
#define INA219_ERR              (-1)

/* ------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------ */

/*
 * Initialise the INA219: write configuration and calibration registers.
 * Returns INA219_OK on success, INA219_ERR on I2C failure.
 */
int8_t ina219_init(void);

/*
 * Read bus voltage and current.
 *
 * vbus_mv_out  : bus voltage in millivolts (unsigned, 0-32000)
 * current_ma_out: signed current in milliamps (positive = charging into load)
 *
 * Returns INA219_OK on success, INA219_ERR on I2C failure or OVF.
 */
int8_t ina219_read(uint16_t *vbus_mv_out, int16_t *current_ma_out);

#endif /* INA219_H_ */
