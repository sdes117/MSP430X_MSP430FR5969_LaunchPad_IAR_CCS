/*
 * ina219.h
 *
 * Abstracted INA219 current/power monitor driver.
 * Each INA219 instance is described by an ina219_t configuration struct,
 * allowing multiple sensors at different I2C addresses to share this driver.
 *
 * Calibration procedure (from datasheet §8.5.1):
 *
 *   Current_LSB = Max_Expected_Current / 2^15          [A/LSB]
 *   Cal         = trunc(0.04096 / (Current_LSB × R_shunt))
 *   Power_LSB   = 20 × Current_LSB                     [W/LSB]
 *
 * Example — 3V3_MSP line (address 0x41):
 *   R_shunt     = 50 mΩ  (two 100 mΩ resistors in parallel)
 *   I_max       = 50 mA
 *   Current_LSB = 50e-3 / 32768 ≈ 1.53 µA → use 50 µA for round numbers
 *   Cal         = trunc(0.04096 / (50e-6 × 0.050)) = 16384
 *   Power_LSB   = 20 × 50 µA = 1 mW/LSB
 *   Config      = 0x019F  (16 V range, PGA=/1 ±40 mV, 12-bit ADC, continuous)
 */

#ifndef INA219_H_
#define INA219_H_

#include <stdint.h>

/* ------------------------------------------------------------------
 * INA219 register addresses (datasheet Table 2)
 * ------------------------------------------------------------------ */
#define INA219_REG_CONFIG   (0x00u)
#define INA219_REG_SHUNT_V  (0x01u)
#define INA219_REG_BUS_V    (0x02u)
#define INA219_REG_POWER    (0x03u)
#define INA219_REG_CURRENT  (0x04u)
#define INA219_REG_CALIB    (0x05u)

/* ------------------------------------------------------------------
 * Configuration register bit fields (datasheet Figure 19)
 * Combine with OR to build a config value.
 * ------------------------------------------------------------------ */
#define INA219_CFG_BRNG_32V  (0x2000u)  /* bus voltage range: 32 V */
#define INA219_CFG_BRNG_16V  (0x0000u)  /* bus voltage range: 16 V */
#define INA219_CFG_PGA_1     (0x0000u)  /* shunt PGA = /1,  ±40 mV */
#define INA219_CFG_PGA_2     (0x0800u)  /* shunt PGA = /2,  ±80 mV */
#define INA219_CFG_PGA_4     (0x1000u)  /* shunt PGA = /4, ±160 mV */
#define INA219_CFG_PGA_8     (0x1800u)  /* shunt PGA = /8, ±320 mV */
#define INA219_CFG_BADC_12   (0x0180u)  /* bus ADC  12-bit (532 µs) */
#define INA219_CFG_SADC_12   (0x0018u)  /* shunt ADC 12-bit (532 µs) */
#define INA219_CFG_MODE_CONT (0x0007u)  /* continuous shunt+bus */

/* ------------------------------------------------------------------
 * Pre-built config values for common scenarios
 * ------------------------------------------------------------------ */

/*
 * Low-current rail (≤50 mA), 50 mΩ shunt, 3.3 V bus:
 *   16 V range, PGA=/1 (±40 mV covers 2.5 mV at 50 mA), 12-bit, continuous
 *   Calibration = 16384, Current_LSB = 50 µA, Power_LSB = 1 mW
 */
#define INA219_CFG_3V3_MSP  (INA219_CFG_BRNG_16V | INA219_CFG_PGA_1 | \
                              INA219_CFG_BADC_12  | INA219_CFG_SADC_12 | \
                              INA219_CFG_MODE_CONT)
#define INA219_CALIB_3V3_MSP    (16384u)
#define INA219_LSB_UA_3V3_MSP   (50)     /* µA per current-register LSB */

/* ------------------------------------------------------------------
 * Device instance descriptor
 * Populate one of these per physical INA219 on the bus.
 * ------------------------------------------------------------------ */
typedef struct {
    uint8_t  addr;            /* I2C 7-bit address                    */
    uint16_t config;          /* value to write to Config register     */
    uint16_t calib;           /* value to write to Calibration register*/
    int32_t  current_lsb_ua;  /* current LSB in µA (sets scale)       */
} ina219_t;

/* ------------------------------------------------------------------
 * Measurement result
 * All fields are valid only when ina219_read() returns INA219_OK.
 * ------------------------------------------------------------------ */
typedef struct {
    int32_t  shunt_uv;    /* shunt voltage, µV, signed (±range × 10 µV/LSB)  */
    uint16_t bus_mv;      /* bus voltage,   mV, unsigned (4 mV/LSB after >>3) */
    int16_t  current_ma;  /* current,       mA, signed                        */
    uint16_t power_mw;    /* power,         mW, unsigned                      */
} ina219_data_t;

/* Return codes */
#define INA219_OK   ( 0)
#define INA219_ERR  (-1)

/* ------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------ */

/*
 * Write Config and Calibration registers.
 * Call once at startup (or after power-loss of the sensor).
 * Returns INA219_OK or INA219_ERR.
 */
int8_t ina219_init(const ina219_t *dev);

/*
 * Read all four data registers into *out.
 * Returns INA219_OK on success, INA219_ERR on any I2C failure.
 */
int8_t ina219_read(const ina219_t *dev, ina219_data_t *out);

#endif /* INA219_H_ */
