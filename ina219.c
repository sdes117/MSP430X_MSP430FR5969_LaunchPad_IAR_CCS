/*
 * ina219.c
 *
 * INA219 I2C current/power monitor driver.
 * See ina219.h for full description.
 *
 * INA219 registers are 16-bit, big-endian (MSB at lower address).
 * supervisor_i2c uses little-endian byte buffers, so byte order is
 * handled explicitly here.
 */

#include "ina219.h"
#include "supervisor_i2c.h"

/* ------------------------------------------------------------------
 * Internal helper: write a 16-bit register (big-endian).
 * ------------------------------------------------------------------ */
static int8_t prv_write_reg16(uint8_t reg, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(value >> 8u);    /* MSB first */
    buf[1] = (uint8_t)(value & 0xFFu);
    return i2c_write_reg(INA219_ADDR, reg, buf, 2u);
}

/* ------------------------------------------------------------------
 * Internal helper: read a 16-bit register (big-endian).
 * ------------------------------------------------------------------ */
static int8_t prv_read_reg16(uint8_t reg, uint16_t *out)
{
    uint8_t buf[2] = {0u, 0u};
    int8_t  rc;

    rc = i2c_read_reg(INA219_ADDR, reg, buf, 2u);
    if (rc == I2C_OK)
    {
        *out = ((uint16_t)buf[0] << 8u) | (uint16_t)buf[1];  /* MSB first */
    }
    return rc;
}

/* ------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------ */

int8_t ina219_init(void)
{
    int8_t rc;

    rc = prv_write_reg16(INA219_REG_CONFIG, INA219_CONFIG_DEFAULT);
    if (rc != I2C_OK)
    {
        return INA219_ERR;
    }

    rc = prv_write_reg16(INA219_REG_CALIB, INA219_CALIB_VALUE);
    if (rc != I2C_OK)
    {
        return INA219_ERR;
    }

    return INA219_OK;
}

int8_t ina219_read(uint16_t *vbus_mv_out, int16_t *current_ma_out)
{
    uint16_t bus_raw;
    uint16_t cur_raw;
    int8_t   rc;

    /* Bus voltage register: bits [15:3] = voltage / 4mV; bit0 = OVF */
    rc = prv_read_reg16(INA219_REG_BUS_V, &bus_raw);
    if (rc != I2C_OK)
    {
        return INA219_ERR;
    }

    if (bus_raw & 0x0001u)
    {
        /* Overflow flag set — reading is invalid */
        return INA219_ERR;
    }

    /* Shift out the CNVR and OVF bits, then scale by 4 mV/LSB */
    *vbus_mv_out = (uint16_t)((bus_raw >> 3u) * INA219_BUS_V_LSB_MV);

    /* Current register: signed 16-bit, units = Current_LSB */
    rc = prv_read_reg16(INA219_REG_CURRENT, &cur_raw);
    if (rc != I2C_OK)
    {
        return INA219_ERR;
    }

    {
        /*
         * Convert to milliamps:
         *   current_ma = raw_signed * Current_LSB_uA / 1000
         * Use int32_t intermediate to avoid 16-bit overflow.
         */
        int16_t  cur_signed = (int16_t)cur_raw;
        int32_t  cur_ua     = (int32_t)cur_signed * (int32_t)INA219_CURRENT_LSB_UA;
        *current_ma_out     = (int16_t)(cur_ua / 1000);
    }

    return INA219_OK;
}
