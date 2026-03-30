/*
 * ina219.c
 *
 * Abstracted INA219 current/power monitor driver.
 * Depends on supervisor_i2c for bus access.
 */

#include "ina219.h"
#include "supervisor_i2c.h"

/* ------------------------------------------------------------------
 * Write a 16-bit register (big-endian) via I2C.
 * ------------------------------------------------------------------ */
static int8_t write_reg16(uint8_t addr, uint8_t reg, uint16_t val)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(val >> 8u);
    buf[1] = (uint8_t)(val & 0xFFu);
    return i2c_write_reg(addr, reg, buf, 2u);
}

/* ------------------------------------------------------------------
 * Read a 16-bit register (big-endian) via I2C.
 * ------------------------------------------------------------------ */
static int8_t read_reg16(uint8_t addr, uint8_t reg, uint16_t *out)
{
    uint8_t buf[2] = { 0u, 0u };
    int8_t  rc = i2c_read_reg(addr, reg, buf, 2u);
    if (rc == I2C_OK)
        *out = (uint16_t)(((uint16_t)buf[0] << 8u) | (uint16_t)buf[1]);
    return rc;
}

/* ------------------------------------------------------------------
 * ina219_init — write Config then Calibration register.
 * ------------------------------------------------------------------ */
int8_t ina219_init(const ina219_t *dev)
{
    if (!dev) return INA219_ERR;

    if (write_reg16(dev->addr, INA219_REG_CONFIG, dev->config) != I2C_OK)
        return INA219_ERR;

    if (write_reg16(dev->addr, INA219_REG_CALIB, dev->calib) != I2C_OK)
        return INA219_ERR;

    return INA219_OK;
}

/* ------------------------------------------------------------------
 * ina219_read — read all four data registers into *out.
 *
 * Register decoding:
 *   Shunt voltage : signed 16-bit, 10 µV/LSB  → result in µV
 *   Bus voltage   : bits [15:3], 4 mV/LSB      → result in mV
 *   Current       : signed 16-bit, current_lsb_ua µA/LSB → result in mA
 *   Power         : unsigned 16-bit, 20×current_lsb_ua µW/LSB → result in mW
 * ------------------------------------------------------------------ */
int8_t ina219_read(const ina219_t *dev, ina219_data_t *out)
{
    uint16_t raw_shunt, raw_bus, raw_current, raw_power;

    if (!dev || !out) return INA219_ERR;

    if (read_reg16(dev->addr, INA219_REG_SHUNT_V, &raw_shunt)  != I2C_OK) return INA219_ERR;
    if (read_reg16(dev->addr, INA219_REG_BUS_V,   &raw_bus)    != I2C_OK) return INA219_ERR;
    if (read_reg16(dev->addr, INA219_REG_CURRENT, &raw_current) != I2C_OK) return INA219_ERR;
    if (read_reg16(dev->addr, INA219_REG_POWER,   &raw_power)  != I2C_OK) return INA219_ERR;

    /* Shunt voltage: signed 16-bit × 10 µV/LSB */
    out->shunt_uv = (int32_t)(int16_t)raw_shunt * 10;

    /* Bus voltage: bits [15:3] hold the measurement, 4 mV/LSB */
    out->bus_mv = (uint16_t)((raw_bus >> 3u) * 4u);

    /* Current: signed 16-bit × current_lsb_ua → µA, divide by 1000 → mA */
    {
        int32_t ua = (int32_t)(int16_t)raw_current * dev->current_lsb_ua;
        out->current_ma = (int16_t)(ua / 1000);
    }

    /* Power: unsigned 16-bit × (20 × current_lsb_ua) → µW, divide by 1000 → mW */
    {
        int32_t uw = (int32_t)raw_power * (20 * dev->current_lsb_ua);
        out->power_mw = (uint16_t)(uw / 1000u);
    }

    return INA219_OK;
}
